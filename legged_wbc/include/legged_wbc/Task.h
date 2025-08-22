//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#pragma once

#include <ocs2_core/Types.h>

#include <utility>

namespace legged {
using namespace ocs2;

/**
 * @brief WBC中的任务定义
 *
 * 这个类用于表示WBC优化问题中的一个任务。
 * 一个任务可以是一个等式约束、一个不等式约束，或者一个二次代价函数的一部分。
 *
 * 形式:
 * - 等式约束: Ax = b
 * - 不等式约束: Dx <= f
 * - 二次代价: 1/2 * x' * (A'A) * x - (A'b)' * x
 *
 * 成员变量:
 * - A: matrix_t a_
 * - b: vector_t b_
 * - D: matrix_t d_
 * - f: vector_t f_
 */
class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  /**
   * @brief 构造函数
   */
  Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

  /**
   * @brief 构造一个空的任务
   * @param numDecisionVars 决策变量的数量
   */
  explicit Task(size_t numDecisionVars)
      : Task(matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0), matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0)) {}

  /**
   * @brief 重载加法运算符，用于堆叠（concatenate）两个任务
   *
   * 这允许将多个任务合并为一个大的任务。
   */
  Task operator+(const Task& rhs) const {
    return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
            concatenateVectors(f_, rhs.f_)};
  }

  /**
   * @brief 重载乘法运算符，用于给任务加权
   *
   * @param rhs 权重系数
   * @return 加权后的新任务
   */
  Task operator*(scalar_t rhs) const {
    return {a_.cols() > 0 ? rhs * a_ : a_,
            b_.cols() > 0 ? rhs * b_ : b_,
            d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_};
  }

  matrix_t a_, d_; // 约束/代价矩阵
  vector_t b_, f_; // 约束/代价向量

  /**
   * @brief 辅助函数，用于垂直拼接两个矩阵
   */
  static matrix_t concatenateMatrices(matrix_t m1, matrix_t m2) {
    if (m1.cols() <= 0) {
      return m2;
    } else if (m2.cols() <= 0) {
      return m1;
    }
    assert(m1.cols() == m2.cols());
    matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
  }

  /**
   * @brief 辅助函数，用于垂直拼接两个向量
   */
  static vector_t concatenateVectors(const vector_t& v1, const vector_t& v2) {
    if (v1.cols() <= 0) {
      return v2;
    } else if (v2.cols() <= 0) {
      return v1;
    }
    assert(v1.cols() == v2.cols());
    vector_t res(v1.rows() + v2.rows());
    res << v1, v2;
    return res;
  }
};

}  // namespace legged
