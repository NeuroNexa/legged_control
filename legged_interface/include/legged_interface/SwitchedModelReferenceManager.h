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

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class SwitchedModelReferenceManager
 * @brief Manages the ModeSchedule and the TargetTrajectories for a switched model (e.g., a legged robot).
 * This class is responsible for adapting the reference trajectories based on a predefined gait schedule
 * and generating swing leg motions. It serves as a high-level manager for the robot's intended movement.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  /**
   * @brief Constructor for SwitchedModelReferenceManager.
   * @param gaitSchedulePtr : A pointer to the gait schedule, which defines the contact patterns over time.
   * @param swingTrajectoryPtr : A pointer to the swing trajectory planner, which generates trajectories for feet in the air.
   */
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  /**
   * @brief Sets the mode schedule for the robot. This is typically called to update the gait sequence.
   * @param modeSchedule : The new mode schedule to be used.
   */
  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  /**
   * @brief Gets the contact flags for a specific time.
   * @param time : The time at which to query the contact flags.
   * @return A vector of booleans indicating if each foot is in contact.
   */
  contact_flag_t getContactFlags(scalar_t time) const;

  /**
   * @brief Gets a pointer to the managed gait schedule.
   * @return A const shared pointer to the GaitSchedule.
   */
  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  /**
   * @brief Gets a pointer to the managed swing trajectory planner.
   * @return A const shared pointer to the SwingTrajectoryPlanner.
   */
  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 protected:
  /**
   * @brief Modifies the reference trajectories based on the current gait and system state.
   * This is the core function where swing trajectories are generated and target trajectories are updated.
   * @param initTime : The initial time of the optimization horizon.
   * @param finalTime : The final time of the optimization horizon.
   * @param initState : The initial state of the robot.
   * @param [out] targetTrajectories : The target trajectories to be modified.
   * @param [out] modeSchedule : The mode schedule to be used for the horizon.
   */
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
