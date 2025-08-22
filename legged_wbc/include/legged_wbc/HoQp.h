//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include "legged_wbc/Task.h"

#include <memory>

namespace legged {
/**
 * @brief 分层优化二次规划 (Hierarchical Optimization Quadratic Program)
 *
 * 这个类实现了一个分层QP问题的求解器。它通过递归的方式处理一系列具有优先级的任务。
 * 每个HoQp对象代表层级中的一个级别。它接收一个任务（Task）和一个指向更高优先级问题（higherProblem）的指针。
 *
 * 它的核心思想是：
 * 1. 首先，它获取更高优先级问题的解空间（通过零空间投影矩阵 Z）。
 * 2. 然后，它将当前级别的任务投影到这个解空间中，形成一个新的、规模更小的QP问题。
 * 3. 它求解这个新的QP问题，得到在不违反更高优先级任务的前提下，对当前任务最优的解。
 * 4. 这个过程可以递归地进行，形成一个任务层级。
 *
 * QP问题形式:
 * min 1/2 * x'Hx + c'x
 * s.t. Dx = f
 *      Gx <= h
 */
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  /**
   * @brief 构造函数 (用于最高优先级的任务)
   * @param task 当前级别的任务
   */
  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  /**
   * @brief 构造函数 (用于非最高优先级的任务)
   * @param task 当前级别的任务
   * @param higherProblem 指向更高优先级问题的共享指针
   */
  HoQp(Task task, HoQpPtr higherProblem);

  // --- Getters ---
  matrix_t getStackedZMatrix() const { return stackedZ_; } // 获取从该层级开始的累积零空间投影矩阵
  Task getStackedTasks() const { return stackedTasks_; } // 获取从该层级开始的累积任务
  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; } // 获取累积的松弛变量解
  vector_t getSolutions() const; // 获取最终的决策变量解

  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  /**
   * @brief 初始化变量
   */
  void initVars();

  /**
   * @brief 构建QP问题
   */
  void formulateProblem();

  /**
   * @brief 求解QP问题
   */
  void solveProblem();

  // --- 构建QP矩阵的辅助函数 ---
  void buildHMatrix(); // 构建代价函数的Hessian矩阵 H
  void buildCVector(); // 构建代价函数的梯度向量 c
  void buildDMatrix(); // 构建等式约束的矩阵 D
  void buildFVector(); // 构建等式约束的向量 f

  /**
   * @brief 构建零空间投影矩阵 Z
   */
  void buildZMatrix();

  /**
   * @brief 堆叠松弛变量解
   */
  void stackSlackSolutions();

  // --- 成员变量 ---
  Task task_; // 当前级别的任务
  Task stackedTasksPrev_, stackedTasks_; // 上一层和当前层的累积任务
  HoQpPtr higherProblem_; // 指向更高优先级问题的指针

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};
  matrix_t stackedZPrev_, stackedZ_; // 上一层和当前层的累积零空间投影矩阵
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  // QP矩阵 (H, c, D, f)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;
  // QP问题的解
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  // 方便计算用的临时矩阵
  matrix_t eyeNvNv_;
  matrix_t zeroNvNx_;
};

}  // namespace legged
