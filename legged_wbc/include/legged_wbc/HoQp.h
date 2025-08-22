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
 * @brief Implements a single level in a hierarchical optimization quadratic program (HoQP).
 *
 * This class is designed to be used in a cascade to solve a sequence of prioritized tasks.
 * Each HoQp object represents one level of the hierarchy. It takes a task (defined as a QP)
 * and an optional pointer to a higher-priority problem.
 *
 * The core idea is to solve the current task's QP within the null space of the higher-priority tasks.
 * This ensures that the solutions of higher-priority tasks are respected as hard constraints while
 * optimizing for the current task.
 *
 * The QP solved at each level is of the form:
 *   min 0.5 * x' * H * x + c' * x
 *   s.t. D * x <= f
 */
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  /**
   * @brief Constructor for the highest-priority problem in the hierarchy.
   * @param task The task to be solved at this level.
   */
  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  /**
   * @brief Constructor for a lower-priority problem.
   * @param task The task to be solved at this level.
   * @param higherProblem A pointer to the next higher-priority problem.
   */
  HoQp(Task task, HoQpPtr higherProblem);

  /** @brief Gets the stacked null space projection matrix from all higher-priority tasks. */
  matrix_t getStackedZMatrix() const { return stackedZ_; }

  /** @brief Gets the stacked task formulation (A, b, D, f) from all higher-priority tasks. */
  Task getStackedTasks() const { return stackedTasks_; }

  /** @brief Gets the stacked solution for the slack variables from all higher-priority tasks. */
  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  /**
   * @brief Gets the final solution for the decision variables.
   * This reconstructs the solution by projecting the current level's solution out of the null space
   * and adding the particular solution from the higher-priority levels.
   * @return The optimal decision variable vector.
   */
  vector_t getSolutions() const {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  /** @brief Gets the total number of slack variables in the stacked problem. */
  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  // Initialization and problem setup
  void initVars();
  void formulateProblem();
  void solveProblem();

  // Methods to build the components of the QP
  void buildHMatrix();   // Builds the Hessian matrix (H)
  void buildCVector();   // Builds the gradient vector (c)
  void buildDMatrix();   // Builds the inequality constraint matrix (D)
  void buildFVector();   // Builds the inequality constraint vector (f)

  // Methods for handling the hierarchy
  void buildZMatrix();          // Builds the null space projection matrix (Z)
  void stackSlackSolutions();   // Stacks the slack variable solutions

  // Task definitions
  Task task_;                 // The task for the current level
  Task stackedTasksPrev_;     // Stacked tasks from previous (higher-priority) levels
  Task stackedTasks_;         // All tasks up to and including the current level

  HoQpPtr higherProblem_;     // Pointer to the next higher-priority problem

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};

  // Matrices for null-space projection
  matrix_t stackedZPrev_, stackedZ_;
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  // QP matrices (H, c, D, f)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;

  // Solution vectors
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;
};

}  // namespace legged
