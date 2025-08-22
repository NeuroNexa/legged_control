//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>

namespace legged {

// 构造函数立即解决当前层级的 QP 问题。
HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  // 初始化变量，如果存在更高优先级的问题，则从中获取数据。
  initVars();
  // 为当前层级构建 QP 矩阵 (H, c, D, f)。
  formulateProblem();
  // 使用 qpOASES 解决 QP 问题。
  solveProblem();
  // 为下一个（较低优先级）问题做准备。
  buildZMatrix();          // 为下一层级计算零空间投影。
  stackSlackSolutions();   // 收集松弛变量的解。
}

void HoQp::initVars() {
  // 获取当前任务的属性。
  numSlackVars_ = task_.d_.rows();
  hasEqConstraints_ = task_.a_.rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  // 如果存在更高优先级的问题，则获取其解和堆叠数据。
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
    xPrev_ = higherProblem_->getSolutions(); // 这是来自更高层级的特定解。
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();

    // 此层级的决策变量位于更高层级的零空间中。
    numDecisionVars_ = stackedZPrev_.cols();
  } else { // 这是最高优先级的问题。
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());

    // 使用空/单位矩阵进行初始化。
    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
    numPrevSlackVars_ = 0;
  }

  // 将当前任务与所有更高优先级的任务合并。
  stackedTasks_ = task_ + stackedTasksPrev_;

  // 初始化方便计算的矩阵 (这些在原版代码中存在，但在当前版本中被移除了，为保持清晰，此处省略)
  // eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);
  // zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);
}

void HoQp::formulateProblem() {
  // 构建 QP 求解器所需的所有矩阵。
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HoQp::buildHMatrix() {
  // Hessian 矩阵 H 代表代价函数：min 0.5 * x' * H * x
  // 对于等式约束 (Ax=b)，代价是 ||Ax - b||^2，因此 H = A'A。
  // 我们将 A 投影到更高优先级任务的零空间中：A_proj = A * Z_prev。
  // 所以，H = (A_proj)' * (A_proj) = Z_prev' * A' * A * Z_prev。
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    // 确保 A_t_A 的所有特征值都是非负的，这可能是由于数值问题引起的。
    matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
  } else {
    zTaTaz.setZero();
  }

  // 完整的 Hessian 矩阵包括松弛变量的项，其权重为 1。
  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)
            << zTaTaz, matrix_t::Zero(numDecisionVars_, numSlackVars_),
                matrix_t::Zero(numSlackVars_, numDecisionVars_), matrix_t::Identity(numSlackVars_, numSlackVars_))
           .finished();
}

void HoQp::buildCVector() {
  // 向量 c 代表代价函数的线性部分：min c' * x
  // 对于等式约束，这来自于 -2 * b' * A * x 项，所以 c = -A' * b。
  // 在替换 x = x_prev + Z_prev * w 后，新变量 w 的线性项变为
  // c = Z_prev' * A' * (A * x_prev - b)。
  vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();
  }

  // 完整的 c 向量也包含松弛变量的零部分。
  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HoQp::buildDMatrix() {
  // 矩阵 D 代表不等式约束：D * x <= f。
  // 我们有两组不等式：
  // 1. 来自更高优先级任务的不等式：D_prev * x <= f_prev
  // 2. 来自当前任务的不等式：D_curr * x <= f_curr
  // 在替换 x = x_prev + Z_prev * w 后，这些变为：
  // 1. D_prev * Z_prev * w <= f_prev - D_prev * x_prev
  // 2. D_curr * Z_prev * w <= f_curr - D_curr * x_prev
  matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;
  } else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);
  }

  // 引入松弛变量 (s)：D*x - s <= f  和  s >= 0。
  // 此处的矩阵是为求解器构建的，其形式为 D*x <= f。
  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)
            << matrix_t::Zero(numSlackVars_, numDecisionVars_), -matrix_t::Identity(numSlackVars_, numSlackVars_), // -s <= 0  (s >= 0)
                stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,                                                 // D_prev * Z * w <= ...
                dCurrZ,                               -matrix_t::Identity(numSlackVars_, numSlackVars_))          // D_curr * Z * w - s <= ...
           .finished();
}

void HoQp::buildFVector() {
  // 向量 f 是不等式约束 D * x <= f 的右侧。
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
  } else {
    fMinusDXPrev = vector_t::Zero(0);
  }

  // 堆叠的 f 向量对应于 D 矩阵的行。
  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

void HoQp::buildZMatrix() {
  // 零空间投影矩阵 Z 是根据等式约束 A*x=b 计算的。
  // 任何解都可以写成 x = x_p + Z*w，其中 x_p 是一个特解，
  // Z 是 A 的零空间的基。
  // 对于层次结构，Z_i = Z_{i-1} * Z_curr，其中 Z_curr 是 A_i * Z_{i-1} 的零空间。
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    // 我们使用一种鲁棒的分解方法 (fullPivLu) 来找到核（零空间）。
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    // 如果没有等式约束，零空间就是整个空间。
    stackedZ_ = stackedZPrev_;
  }
}

void HoQp::solveProblem() {
  // 使用 qpOASES 库来解决 QP 问题。
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC(); // 使用为 MPC 应用优化的设置。
  options.printLevel = qpOASES::PL_LOW; // 抑制大部分求解器的输出。
  qpProblem.setOptions(options);
  int nWsr = 20; // 工作集重新计算的次数。

  // 初始化并解决问题。
  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);

  // 获取解。
  qpProblem.getPrimalSolution(qpSol.data());

  // 将解分为决策变量和松弛变量。
  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HoQp::stackSlackSolutions() {
  // 将此层级的松弛解与更高层级的松弛解连接起来。
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

}  // namespace legged
