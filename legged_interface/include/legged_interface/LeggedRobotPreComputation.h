/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

#include "legged_interface/constraint/EndEffectorLinearConstraint.h"
#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class LeggedRobotPreComputation
 * @brief A class for caching computations that are shared across different modules of the optimal control problem.
 *
 * In OCS2, the PreComputation module is called by the solver before evaluating the cost and constraints at each
 * time step. This allows for performing expensive computations once and caching the results (e.g., forward kinematics)
 * so they can be reused by various cost and constraint terms without redundant calculations.
 *
 * This specific implementation updates the Pinocchio model with the current state and caches the configuration
 * for the normal velocity constraints on the end-effectors.
 */
class LeggedRobotPreComputation : public PreComputation {
 public:
  /**
   * @brief Constructor for LeggedRobotPreComputation.
   * @param pinocchioInterface : The Pinocchio interface for the robot model.
   * @param info : The centroidal model information.
   * @param swingTrajectoryPlanner : The planner for swing foot trajectories.
   * @param settings : The robot model settings.
   */
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);

  ~LeggedRobotPreComputation() override = default;

  LeggedRobotPreComputation* clone() const override { return new LeggedRobotPreComputation(*this); }

  /**
   * @brief This method is called by the solver to trigger the pre-computation.
   * @param request : A set of flags indicating which modules require the pre-computation.
   * @param t : The current time.
   * @param x : The current state vector.
   * @param u : The current input vector.
   */
  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  /** @brief Gets the cached configurations for the end-effector normal velocity constraints. */
  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  /** @brief Gets a mutable reference to the cached Pinocchio interface. */
  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  /** @brief Gets a constant reference to the cached Pinocchio interface. */
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other);

 private:
  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  const ModelSettings settings_;

  // Cached data
  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

}  // namespace legged_robot
}  // namespace ocs2
