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

#include "legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "legged_interface/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      // Initialize the underlying linear constraint. It is configured to constrain 1 dimension (the normal velocity).
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Standard copy constructor
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {
  // The constraint is active if the foot is NOT in contact (i.e., it's a swing foot).
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NormalVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  // Cast the pre-computation to the legged robot specific type.
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // Dynamically configure the underlying linear constraint with the parameters for the current time step.
  // These parameters (e.g., the desired normal velocity) are computed in the LeggedRobotPreComputation module.
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  // The constraint value is computed by the now-configured underlying EndEffectorLinearConstraint object.
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  // Cast the pre-computation to the legged robot specific type.
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  // Dynamically configure the underlying linear constraint with the parameters for the current time step.
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);

  // The linear approximation is computed by the now-configured underlying EndEffectorLinearConstraint object.
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace ocs2
