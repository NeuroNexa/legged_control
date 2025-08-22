/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "legged_interface/SwitchedModelReferenceManager.h"
#include "legged_interface/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 末端执行器法向速度约束 (CppAd版本)
 *
 * 这是一个专门用于约束摆动腿在垂直（法向）方向上速度的类。
 * 它确保摆动腿的垂直速度跟踪由 `SwingTrajectoryPlanner` 生成的参考速度。
 *
 * 这个类在内部使用了一个 `EndEffectorLinearConstraint` 对象来实现。
 * 它通过查询 `SwitchedModelReferenceManager` (其中包含了 `SwingTrajectoryPlanner`)
 * 来获取期望的法向速度 `v_z_ref`，然后构建一个如下形式的线性约束：
 *
 * g(v_ee) = [0, 0, 1] * v_ee - v_z_ref = 0
 *
 * 这等价于 v_ee_z - v_z_ref = 0。
 *
 * "CppAd" 后缀表示这个约束的导数是通过CppAd库进行自动微分生成的，这在OCS2中是一种常见的做法。
 */
class NormalVelocityConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * @brief 构造函数
   * @param referenceManager 参考管理器，用于获取摆动腿轨迹。
   * @param endEffectorKinematics 目标末端执行器的运动学接口。
   * @param contactPointIndex 接触点的索引。
   */
  NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex);

  ~NormalVelocityConstraintCppAd() override = default;
  NormalVelocityConstraintCppAd* clone() const override { return new NormalVelocityConstraintCppAd(*this); }

  /**
   * @brief 检查约束在给定时间是否激活
   *
   * 只有当腿处于摆动相时，这个约束才激活。
   */
  bool isActive(scalar_t time) const override;

  // --- OCS2 StateInputConstraint 接口的实现 ---
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
  const size_t contactPointIndex_;
};

}  // namespace legged_robot
}  // namespace ocs2
