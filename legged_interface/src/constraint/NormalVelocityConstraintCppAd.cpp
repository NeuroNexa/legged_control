/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "legged_interface/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief NormalVelocityConstraintCppAd 构造函数
 */
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear), // 声明这是一个线性约束
      referenceManagerPtr_(&referenceManager),
      // 创建一个内部的EndEffectorLinearConstraint实例，约束数量为1（只约束Z轴速度）
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief NormalVelocityConstraintCppAd 拷贝构造函数
 */
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 检查约束是否激活
 *
 * 当腿处于摆动相时，此约束激活。
 */
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的值
 */
vector_t NormalVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  // 从预计算模块中获取当前时刻的线性约束配置
  // 这个配置是由 SwingTrajectoryPlanner 生成的，包含了期望的Z轴速度
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  // 调用内部的EndEffectorLinearConstraint来计算约束值
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的线性逼近
 */
VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  // 从预计算模块中获取当前时刻的线性约束配置
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  // 调用内部的EndEffectorLinearConstraint来计算线性逼近
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace ocs2
