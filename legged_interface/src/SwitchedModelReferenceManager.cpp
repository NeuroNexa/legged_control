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

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),  // Initialize base class
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  // Set the mode schedule in the base class
  ReferenceManager::setModeSchedule(modeSchedule);
  // Also set the mode schedule in the associated gait schedule object
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  // Get the mode at the given time and convert it to a contact flag vector
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& /*targetTrajectories*/, ModeSchedule& modeSchedule) {
  // Get the mode schedule for the upcoming time horizon
  const auto timeHorizon = finalTime - initTime;
  // Extend the mode schedule horizon to ensure the planner has future gait information
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  // Update the swing trajectory planner with the new mode schedule and terrain information
  // Note: The terrain is assumed to be flat here. For uneven terrain, this would need to be updated.
  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace legged_robot
}  // namespace ocs2
