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

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "legged_interface/SwitchedModelReferenceManager.h"
#include "legged_interface/constraint/EndEffectorLinearConstraint.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class NormalVelocityConstraintCppAd
 * @brief This class constrains the velocity of a foot in the direction normal to the contact surface.
 * It is typically used for swing feet to enforce a desired touchdown velocity, contributing to a smooth landing.
 * This class is a specialization of the more general EndEffectorLinearConstraint, configured to
 * constrain a single dimension of velocity. It uses CppAD for automatic differentiation.
 * The constraint is active only when the reference manager indicates that the foot is NOT in contact.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class NormalVelocityConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor for the NormalVelocityConstraintCppAd.
   * @param referenceManager : Switched model ReferenceManager to check for contact.
   * @param endEffectorKinematics : The kinematic interface to the target end-effector.
   * @param contactPointIndex : The 3-DOF contact index this constraint applies to.
   */
  NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex);

  ~NormalVelocityConstraintCppAd() override = default;
  NormalVelocityConstraintCppAd* clone() const override { return new NormalVelocityConstraintCppAd(*this); }

  /**
   * @brief Checks if the constraint is active at a given time (i.e., if the foot is in a swing phase).
   * @param time : The time to check.
   * @return True if the constraint is active, false otherwise.
   */
  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_; // The underlying linear constraint object
  const size_t contactPointIndex_;
};

}  // namespace legged_robot
}  // namespace ocs2
