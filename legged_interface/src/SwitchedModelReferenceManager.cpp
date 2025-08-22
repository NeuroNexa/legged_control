/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief SwitchedModelReferenceManager 构造函数
 */
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()), // 初始化基类
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 设置模式序列
 *
 * 同时更新基类和内部gaitSchedulePtr_中的模式序列。
 */
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 获取指定时间的接触标志
 */
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  // 从模式序列中获取当前时间的模式ID，然后转换为接触标志向量
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 修改参考轨迹
 *
 * 这个函数在每次设置新的目标轨迹时被调用。
 * 它的主要作用是根据新的时间窗口，从步态调度器中获取相应的步态序列，
 * 然后调用摆动腿轨迹规划器来更新摆动腿的Z轴轨迹。
 */
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  // 计算MPC的时间窗口长度
  const auto timeHorizon = finalTime - initTime;
  // 从步态调度器获取覆盖整个MPC时间窗口（并向前后各延伸一个窗口长度）的步态序列
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  // 假设地形是平坦的（高度为0）
  const scalar_t terrainHeight = 0.0;
  // 更新摆动腿轨迹规划器，它会根据新的步态序列生成新的摆动腿轨迹
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace legged_robot
}  // namespace ocs2
