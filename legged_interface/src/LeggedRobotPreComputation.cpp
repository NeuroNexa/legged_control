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

#include "legged_interface/LeggedRobotPreComputation.h"

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      settings_(std::move(settings)) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Standard copy constructor
LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      info_(rhs.info_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      settings_(rhs.settings_) {
  eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  // If the request doesn't need any pre-computation, return early.
  if (!request.containsAny(Request::Cost | Request::Constraint | Request::SoftConstraint)) {
    return;
  }

  // --- Cache data for the normal velocity constraint ---
  // This lambda creates a configuration for the EndEffectorLinearConstraint.
  // The constraint is on the normal velocity (z-axis) of the foot.
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    // The desired normal velocity is retrieved from the swing trajectory planner.
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    // Optionally, add a position term to the constraint to penalize height errors.
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  // If constraints are requested, compute and cache the config for each foot.
  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
    }
  }

  // --- Update Pinocchio model ---
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);

  // If the solver requires derivatives (approximations), we need to compute the full kinematics,
  // Jacobians, and centroidal dynamics derivatives.
  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  } else {
    // Otherwise, just compute forward kinematics and frame placements.
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
