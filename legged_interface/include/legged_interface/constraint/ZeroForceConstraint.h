/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 零接触力约束
 *
 * 这是一个等式约束，用于确保当一条腿处于摆动相（swing phase）时，
 * 其足端的接触力（ground reaction force）为零。
 *
 * 约束的形式为: F_contact = 0
 *
 * 这个约束只在对应的腿被`SwitchedModelReferenceManager`确定为处于摆动相时才激活。
 */
class ZeroForceConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief 构造函数
   * @param referenceManager 参考管理器，用于查询步态信息。
   * @param contactPointIndex 接触点的索引。
   * @param info 质心模型信息。
   */
  ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex, CentroidalModelInfo info);

  ~ZeroForceConstraint() override = default;
  ZeroForceConstraint* clone() const override { return new ZeroForceConstraint(*this); }

  /**
   * @brief 检查约束在给定时间是否激活
   *
   * 只有当腿处于摆动相时，这个约束才激活。
   */
  bool isActive(scalar_t time) const override;

  // --- OCS2 StateInputConstraint 接口的实现 ---
  size_t getNumConstraints(scalar_t time) const override { return 3; } // 3个约束 (Fx, Fy, Fz)
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ZeroForceConstraint(const ZeroForceConstraint& other) = default;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;
};

}  // namespace legged_robot
}  // namespace ocs2
