/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/initialization/LeggedRobotInitializer.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_legged_robot/common/utils.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief LeggedRobotInitializer 构造函数
 */
LeggedRobotInitializer::LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                                               bool extendNormalizedMomentum)
    : info_(std::move(info)), referenceManagerPtr_(&referenceManager), extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer* LeggedRobotInitializer::clone() const {
  return new LeggedRobotInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算初始的输入和下一个状态
 */
void LeggedRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  // 获取当前时间的接触状态
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);

  // 计算一个仅用于补偿重力的输入作为初始猜测。
  // 这个输入会将总重力平均分配到所有接触的腿上。
  input = weightCompensatingInput(info_, contactFlags);

  // 将下一个状态的初始猜测设为当前状态
  nextState = state;

  // 根据配置，选择是否将归一化动量的初始猜测设为零。
  if (!extendNormalizedMomentum_) {
    centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}

}  // namespace legged_robot
}  // namespace ocs2
