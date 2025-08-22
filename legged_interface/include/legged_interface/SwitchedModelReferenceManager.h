/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

//
// 注意：这个文件的命名空间是 ocs2::legged_robot，而不是 legged。
// 这是因为它是从OCS2的legged_robot包中修改而来的。
//
namespace ocs2 {
namespace legged_robot {

/**
 * @brief 切换模型的参考管理器
 *
 * 该类继承自 `ReferenceManager`，专门用于处理腿式机器人这种具有切换模型（接触/摆动）的系统。
 * 它负责管理步态序列（ModeSchedule）和目标轨迹（TargetTrajectories），
 * 并在需要时（例如，当接收到新的目标指令时）修改它们。
 * 一个关键的功能是，它会根据当前的步态序列，调用 `SwingTrajectoryPlanner` 来生成摆动腿的轨迹，
 * 并将这些轨迹作为中间目标添加到 `TargetTrajectories` 中。
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  /**
   * @brief 构造函数
   * @param gaitSchedulePtr 指向步态调度器的共享指针
   * @param swingTrajectoryPtr 指向摆动腿轨迹规划器的共享指针
   */
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  /**
   * @brief 设置新的模式序列（步态）
   * @param modeSchedule 新的步态序列
   */
  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  /**
   * @brief 获取指定时间的接触标志
   * @param time 查询的时间
   * @return 一个布尔向量，表示每个脚在该时刻是否接触地面
   */
  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 protected:
  /**
   * @brief 修改参考轨迹
   *
   * 这是`ReferenceManager`的核心虚函数。当目标轨迹被更新时，此函数被调用。
   * 它会根据新的步态序列和目标，重新规划摆动腿的轨迹，并将其整合到`targetTrajectories`中。
   * @param initTime 初始时间
   * @param finalTime 最终时间
   * @param initState 初始状态
   * @param targetTrajectories [in/out] 待修改的目标轨迹
   * @param modeSchedule [in/out] 待修改的模式序列
   */
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
