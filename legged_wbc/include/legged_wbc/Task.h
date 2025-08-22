//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include <ocs2_core/Types.h>
#include <utility>

namespace legged {
using namespace ocs2;

/**
 * @class Task
 * @brief A data structure for defining a task in the whole-body control problem.
 *
 * A task represents a set of linear constraints, which can be either equality constraints,
 * inequality constraints, or a combination of both.
 *
 * - Equality constraints are of the form: A * x = b
 * - Inequality constraints are of the form: D * x <= f
 *
 * This class provides a convenient way to store these matrices and vectors and to
 * combine multiple tasks into a single larger task.
 */
class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  /**
   * @brief Constructor for a task.
   * @param a The equality constraint matrix A.
   * @param b The equality constraint vector b.
   * @param d The inequality constraint matrix D.
   * @param f The inequality constraint vector f.
   */
  Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

  /**
   * @brief Constructor for an empty task with a specified number of decision variables.
   * @param numDecisionVars The number of columns for the constraint matrices.
   */
  explicit Task(size_t numDecisionVars)
      : Task(matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0), matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0)) {}

  /**
   * @brief Overloads the + operator to concatenate two tasks.
   * This allows for easily combining multiple constraints into a single task.
   * @param rhs The task to be added.
   * @return A new task containing the combined constraints.
   */
  Task operator+(const Task& rhs) const {
    return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
            concatenateVectors(f_, rhs.f_)};
  }

  /**
   * @brief Overloads the * operator to scale a task by a scalar.
   * @param rhs The scalar to multiply by.
   * @return A new task with scaled constraints.
   */
  Task operator*(scalar_t rhs) const {
    return {a_.cols() > 0 ? rhs * a_ : a_, b_.cols() > 0 ? rhs * b_ : b_, d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_};
  }

  matrix_t a_, d_;  //!< A and D matrices for equality (Ax=b) and inequality (Dx<=f) constraints.
  vector_t b_, f_;  //!< b and f vectors for equality and inequality constraints.

  /**
   * @brief A static helper function to vertically concatenate two matrices.
   * @param m1 The first matrix.
   * @param m2 The second matrix.
   * @return The concatenated matrix.
   */
  static matrix_t concatenateMatrices(const matrix_t& m1, const matrix_t& m2) {
    if (m1.rows() == 0) {
      return m2;
    }
    if (m2.rows() == 0) {
      return m1;
    }
    assert(m1.cols() == m2.cols());
    matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
  }

  /**
   * @brief A static helper function to vertically concatenate two vectors.
   * @param v1 The first vector.
   * @param v2 The second vector.
   * @return The concatenated vector.
   */
  static vector_t concatenateVectors(const vector_t& v1, const vector_t& v2) {
    if (v1.rows() == 0) {
      return v2;
    }
    if (v2.rows() == 0) {
      return v1;
    }
    vector_t res(v1.rows() + v2.rows());
    res << v1, v2;
    return res;
  }
};

}  // namespace legged
