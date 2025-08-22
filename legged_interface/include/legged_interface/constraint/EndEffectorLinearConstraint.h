/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 末端执行器的线性约束
 *
 * 定义了一个通用的、作用于末端执行器位置(x_ee)和线速度(v_ee)的线性约束。
 * 约束的形式为: g(x_ee, v_ee) = A_x * x_ee + A_v * v_ee + b
 *
 * 这个类是其他更具体约束（如ZeroVelocityConstraint, NormalVelocityConstraint）的基石。
 * 通过设置不同的系数矩阵 A_x, A_v 和向量 b，可以实现不同类型的约束。
 * - 如果只想约束位置，将 A_v 设为空矩阵。
 * - 如果只想约束速度，将 A_x 设为空矩阵。
 */
class EndEffectorLinearConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief 线性约束的系数配置
   * g(x_ee, v_ee) = A_x * x_ee + A_v * v_ee + b
   */
  struct Config {
    vector_t b;
    matrix_t Ax;
    matrix_t Av;
  };

  /**
   * @brief 构造函数
   * @param endEffectorKinematics 目标末端执行器的运动学接口。
   * @param numConstraints 约束的数量 (例如，如果约束是3维向量，则为3)。
   * @param config 约束系数。
   */
  EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numConstraints,
                              Config config = Config());

  ~EndEffectorLinearConstraint() override = default;
  EndEffectorLinearConstraint* clone() const override { return new EndEffectorLinearConstraint(*this); }

  /** 设置新的约束系数 */
  void configure(Config&& config);
  void configure(const Config& config) { this->configure(Config(config)); }

  /** 获取底层的末端执行器运动学接口 */
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  // --- OCS2 StateInputConstraint 接口的实现 ---
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t numConstraints_;
  Config config_;
};

}  // namespace legged_robot
}  // namespace ocs2
