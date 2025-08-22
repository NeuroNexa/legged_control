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
 * @brief 末端执行器零速度约束 (CppAd版本)
 *
 * 这是一个专门用于约束支撑腿（stance leg）足端速度为零的类。
 *
 * 这个类在内部使用了一个 `EndEffectorLinearConstraint` 对象来实现。
 * 它构建了一个如下形式的线性约束：
 *
 * g(v_ee) = I * v_ee + 0 = 0  (其中 I 是 3x3 单位矩阵)
 *
 * 这等价于 v_ee = 0。
 *
 * 这个约束只在对应的腿被`SwitchedModelReferenceManager`确定为处于支撑相时才激活。
 * "CppAd" 后缀表示这个约束的导数是通过CppAd库进行自动微分生成的。
 */
class ZeroVelocityConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * @brief 构造函数
   * @param referenceManager 参考管理器，用于查询步态信息。
   * @param endEffectorKinematics 目标末端执行器的运动学接口。
   * @param contactPointIndex 接触点的索引。
   * @param config (可选) 线性约束的配置。
   */
  ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                              const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                              EndEffectorLinearConstraint::Config config = EndEffectorLinearConstraint::Config());

  ~ZeroVelocityConstraintCppAd() override = default;
  ZeroVelocityConstraintCppAd* clone() const override { return new ZeroVelocityConstraintCppAd(*this); }

  /**
   * @brief 检查约束在给定时间是否激活
   *
   * 只有当腿处于支撑相时，这个约束才激活。
   */
  bool isActive(scalar_t time) const override;

  // --- OCS2 StateInputConstraint 接口的实现 ---
  size_t getNumConstraints(scalar_t time) const override { return 3; } // 3个约束 (vx, vy, vz)
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
  const size_t contactPointIndex_;
};

}  // namespace legged_robot
}  // namespace ocs2
