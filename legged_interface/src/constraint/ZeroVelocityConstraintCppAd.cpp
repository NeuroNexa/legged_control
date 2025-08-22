/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/constraint/ZeroVelocityConstraintCppAd.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief ZeroVelocityConstraintCppAd 构造函数
 */
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t contactPointIndex, EndEffectorLinearConstraint::Config config)
    : StateInputConstraint(ConstraintOrder::Linear), // 声明这是一个线性约束
      referenceManagerPtr_(&referenceManager),
      // 创建一个内部的EndEffectorLinearConstraint实例，约束数量为3 (vx, vy, vz)
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 3, std::move(config))),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief ZeroVelocityConstraintCppAd 拷贝构造函数
 */
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs)
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
 * 当腿处于支撑相时，此约束激活。
 */
bool ZeroVelocityConstraintCppAd::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的值
 *
 * 直接调用内部的EndEffectorLinearConstraint来计算约束值。
 * 对于零速度约束，其内部配置通常是 A_v = I, A_x = 0, b = 0，
 * 所以 getValue() 返回的就是末端执行器的速度 v_ee。
 */
vector_t ZeroVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的线性逼近
 *
 * 直接调用内部的EndEffectorLinearConstraint来计算线性逼近。
 */
VectorFunctionLinearApproximation ZeroVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace ocs2
