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
 * @brief 用于定义全身控制问题中任务的数据结构。
 *
 * 一个任务代表一组线性约束，可以是等式约束、不等式约束或两者的组合。
 *
 * - 等式约束形式为：A * x = b
 * - 不等式约束形式为：D * x <= f
 *
 * 此类提供了一种方便的方式来存储这些矩阵和向量，并将多个任务组合成一个更大的任务。
 */
class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  /**
   * @brief 任务的构造函数。
   * @param a 等式约束矩阵 A。
   * @param b 等式约束向量 b。
   * @param d 不等式约束矩阵 D。
   * @param f 不等式约束向量 f。
   */
  Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

  /**
   * @brief 具有指定决策变量数量的空任务的构造函数。
   * @param numDecisionVars 约束矩阵的列数。
   */
  explicit Task(size_t numDecisionVars)
      : Task(matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0), matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0)) {}

  /**
   * @brief 重载 + 运算符以连接两个任务。
   * 这允许轻松地将多个约束组合成单个任务。
   * @param rhs 要添加的任务。
   * @return 包含组合约束的新任务。
   */
  Task operator+(const Task& rhs) const {
    return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
            concatenateVectors(f_, rhs.f_)};
  }

  /**
   * @brief 重载 * 运算符以按标量缩放任务。
   * @param rhs 要乘以的标量。
   * @return 具有缩放约束的新任务。
   */
  Task operator*(scalar_t rhs) const {
    return {a_.cols() > 0 ? rhs * a_ : a_, b_.cols() > 0 ? rhs * b_ : b_, d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_};
  }

  matrix_t a_, d_;  //!< 等式(Ax=b)和不等式(Dx<=f)约束的 A 和 D 矩阵。
  vector_t b_, f_;  //!< 等式和不等式约束的 b 和 f 向量。

  /**
   * @brief 垂直连接两个矩阵的静态辅助函数。
   * @param m1 第一个矩阵。
   * @param m2 第二个矩阵。
   * @return 连接后的矩阵。
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
   * @brief 垂直连接两个向量的静态辅助函数。
   * @param v1 第一个向量。
   * @param v2 第二个向量。
   * @return 连接后的向量。
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
