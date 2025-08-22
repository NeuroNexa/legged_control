//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>

namespace legged {

/**
 * @brief HoQp 构造函数
 *
 * 这是HoQp求解器的核心。它接收当前级别的任务和指向更高优先级问题的指针。
 * 它会自动执行以下步骤：
 * 1. 初始化变量 (initVars)
 * 2. 构建QP问题 (formulateProblem)
 * 3. 求解QP问题 (solveProblem)
 * 4. 为下一层级准备数据（buildZMatrix, stackSlackSolutions）
 */
HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();
  formulateProblem();
  solveProblem();
  buildZMatrix();
  stackSlackSolutions();
}

/**
 * @brief 初始化变量
 */
void HoQp::initVars() {
  // 获取当前任务的维度信息
  numSlackVars_ = task_.d_.rows(); // 不等式约束的数量，也即是松弛变量的数量
  hasEqConstraints_ = task_.a_.rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  // 如果存在更高优先级的问题，则从它那里获取累积的解、零空间矩阵等信息
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
    xPrev_ = higherProblem_->getSolutions();
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();
    numDecisionVars_ = stackedZPrev_.cols();
  } else { // 如果是最高优先级的问题
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());
    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_); // 初始的零空间是整个空间
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
    numPrevSlackVars_ = 0;
  }
  // 将当前任务与之前层级的任务堆叠起来
  stackedTasks_ = task_ + stackedTasksPrev_;
}

/**
 * @brief 构建QP问题
 */
void HoQp::formulateProblem() {
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

/**
 * @brief 构建Hessian矩阵 H
 */
void HoQp::buildHMatrix() {
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);
  if (hasEqConstraints_) {
    // H = Z_prev' * A' * A * Z_prev
    matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
  } else {
    zTaTaz.setZero();
  }
  // 最终的H矩阵包含了决策变量和松弛变量两部分
  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_) << zTaTaz, matrix_t::Zero(numDecisionVars_, numSlackVars_),
        matrix_t::Zero(numSlackVars_, numDecisionVars_), matrix_t::Identity(numSlackVars_, numSlackVars_)).finished();
}

/**
 * @brief 构建梯度向量 c
 */
void HoQp::buildCVector() {
  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    // c = Z_prev' * A' * (A * x_prev - b)
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();
  }
  // 最终的c向量也包含决策变量和松弛变量两部分
  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, vector_t::Zero(numSlackVars_)).finished();
}

/**
 * @brief 构建不等式约束矩阵 D
 */
void HoQp::buildDMatrix() {
  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;
  } else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);
  }
  // 将当前层级和之前层级的不等式约束堆叠起来
  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)
            << matrix_t::Zero(numSlackVars_, numDecisionVars_), -matrix_t::Identity(numSlackVars_, numSlackVars_),
               stackedTasksPrev_.d_ * stackedZPrev_, matrix_t::Zero(numPrevSlackVars_, numSlackVars_),
               dCurrZ, -matrix_t::Identity(numSlackVars_, numSlackVars_)).finished();
}

/**
 * @brief 构建不等式约束向量 f
 */
void HoQp::buildFVector() {
  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
  } else {
    fMinusDXPrev = vector_t::Zero(0);
  }
  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << vector_t::Zero(numSlackVars_),
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev).finished();
}

/**
 * @brief 构建零空间投影矩阵 Z
 */
void HoQp::buildZMatrix() {
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    // Z_curr = Z_prev * ker(A_curr * Z_prev)
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    stackedZ_ = stackedZPrev_;
  }
}

/**
 * @brief 使用qpOASES求解QP问题
 */
void HoQp::solveProblem() {
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int nWsr = 20; // 最大迭代次数

  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);
  qpProblem.getPrimalSolution(qpSol.data());

  decisionVarsSolutions_ = qpSol.head(numDecisionVars_); // 决策变量的解
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);     // 松弛变量的解
}

/**
 * @brief 堆叠松弛变量的解，为下一层级做准备
 */
void HoQp::stackSlackSolutions() {
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

/**
 * @brief 获取最终解
 */
vector_t HoQp::getSolutions() const {
  // x_final = x_prev + Z_prev * w
  // 其中 w 是在零空间中的解 (decisionVarsSolutions_)
  return xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
}

}  // namespace legged
