/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

// 该文件位于 legged_interface 中，但为了与 ocs2 框架保持一致，命名空间为 ocs2::legged_robot。
namespace ocs2 {
namespace legged_robot {

/**
 * @class SwitchedModelReferenceManager
 * @brief 管理切换模型（例如足式机器人）的模式计划和目标轨迹。
 * 此类负责根据预定义的步态计划调整参考轨迹并生成摆动腿的运动。
 * 它充当机器人预期运动的高级管理器。
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  /**
   * @brief SwitchedModelReferenceManager 的构造函数。
   * @param gaitSchedulePtr : 指向步态计划的指针，该计划定义了随时间变化的接触模式。
   * @param swingTrajectoryPtr : 指向摆动轨迹规划器的指针，该规划器生成空中脚的轨迹。
   */
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  /**
   * @brief 设置机器人的模式计划。通常调用此函数来更新步态序列。
   * @param modeSchedule : 要使用的新模式计划。
   */
  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  /**
   * @brief 获取特定时间的接触标志。
   * @param time : 查询接触标志的时间。
   * @return 一个布尔向量，指示每只脚是否接触。
   */
  contact_flag_t getContactFlags(scalar_t time) const;

  /**
   * @brief 获取指向所管理的步态计划的指针。
   * @return 指向 GaitSchedule 的 const 共享指针。
   */
  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  /**
   * @brief 获取指向所管理的摆动轨迹规划器的指针。
   * @return 指向 SwingTrajectoryPlanner 的 const 共享指针。
   */
  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 protected:
  /**
   * @brief 根据当前的步态和系统状态修改参考轨迹。
   * 这是生成摆动轨迹和更新目标轨迹的核心函数。
   * @param initTime : 优化时域的初始时间。
   * @param finalTime : 优化时域的最终时间。
   * @param initState : 机器人的初始状态。
   * @param [out] targetTrajectories : 要修改的目标轨迹。
   * @param [out] modeSchedule : 用于该时域的模式计划。
   */
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
