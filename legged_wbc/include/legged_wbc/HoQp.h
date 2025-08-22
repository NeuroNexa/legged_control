//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include "legged_wbc/Task.h"

#include <memory>

namespace legged {
/**
 * @class HoQp
 * @brief 实现分层优化二次规划（HoQP）中的单个层级。
 *
 * 此类设计用于级联解决一系列有优先级的任务。
 * 每个 HoQp 对象代表层次结构中的一个层级。它接受一个任务（定义为一个 QP）
 * 和一个可选的指向更高优先级问题的指针。
 *
 *核心思想是在更高优先级任务的零空间内解决当前任务的 QP。
 * 这确保了在为当前任务进行优化时，更高优先级任务的解被作为硬约束来遵守。
 *
 * 每个层级解决的 QP 形式如下：
 *   min 0.5 * x' * H * x + c' * x
 *   s.t. D * x <= f
 */
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  /**
   * @brief 层次结构中最高优先级问题的构造函数。
   * @param task 在此层级要解决的任务。
   */
  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  /**
   * @brief 较低优先级问题的构造函数。
   * @param task 在此层级要解决的任务。
   * @param higherProblem 指向下一个更高优先级问题的指针。
   */
  HoQp(Task task, HoQpPtr higherProblem);

  /** @brief 获取来自所有更高优先级任务的堆叠零空间投影矩阵。 */
  matrix_t getStackedZMatrix() const { return stackedZ_; }

  /** @brief 获取来自所有更高优先级任务的堆叠任务公式（A, b, D, f）。 */
  Task getStackedTasks() const { return stackedTasks_; }

  /** @brief 获取来自所有更高优先级任务的堆叠松弛变量解。 */
  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  /**
   * @brief 获取决策变量的最终解。
   * 通过将当前层级的解投影出零空间并加上来自更高优先级层级的特定解来重构解。
   * @return 最优决策变量向量。
   */
  vector_t getSolutions() const {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  /** @brief 获取堆叠问题中松弛变量的总数。 */
  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  // 初始化和问题设置
  void initVars();
  void formulateProblem();
  void solveProblem();

  // 构建 QP 组件的方法
  void buildHMatrix();   // 构建 Hessian 矩阵 (H)
  void buildCVector();   // 构建梯度向量 (c)
  void buildDMatrix();   // 构建不等式约束矩阵 (D)
  void buildFVector();   // 构建不等式约束向量 (f)

  // 处理层次结构的方法
  void buildZMatrix();          // 构建零空间投影矩阵 (Z)
  void stackSlackSolutions();   // 堆叠松弛变量解

  // 任务定义
  Task task_;                 // 当前层级的任务
  Task stackedTasksPrev_;     // 来自先前（更高优先级）层级的堆叠任务
  Task stackedTasks_;         // 直到并包括当前层级的所有任务

  HoQpPtr higherProblem_;     // 指向下一个更高优先级问题的指针

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};

  // 用于零空间投影的矩阵
  matrix_t stackedZPrev_, stackedZ_;
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  // QP 矩阵 (H, c, D, f)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;

  // 解向量
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;
};

}  // namespace legged
