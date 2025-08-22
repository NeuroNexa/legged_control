/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/constraint/ZeroForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief ZeroForceConstraint 构造函数
 */
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,
                                         CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Linear), // 声明这是一个线性约束
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 检查约束是否激活
 *
 * 当腿处于摆动相时，此约束激活。
 */
bool ZeroForceConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的值
 *
 * 约束函数 g(x,u) = F_contact。由于目标是 F_contact = 0，
 * 所以这个函数直接返回从输入向量u中提取的接触力。
 */
vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  return centroidal_model::getContactForces(input, contactPointIndex_, info_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的线性逼近
 *
 * 返回 g(x,u) 的值以及其对于状态x和输入u的雅可比矩阵 (偏导数)。
 */
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp); // 约束值

  // 接触力是输入u的一部分，与状态x无关，所以 df/dx = 0。
  approx.dfdx = matrix_t::Zero(3, state.size());

  // 计算 df/du。因为 F_contact 是 u 的一个分量，所以这是一个简单的选择矩阵。
  // 例如，如果 F_contact 是 u 的前3个元素，则 df/du = [I_3x3, 0_3x(N-3)]。
  approx.dfdu = matrix_t::Zero(3, input.size());
  approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = vector_t::Ones(3);
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
