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

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class ZeroForceConstraint
 * @brief This constraint enforces that the contact force of a specific foot is zero.
 * It is used for feet that are in a swing phase, ensuring they do not exert any forces.
 * The constraint is active only when the reference manager indicates that the foot is not in contact.
 */
class ZeroForceConstraint final : public StateInputConstraint {
 public:
  /**
   * @brief Constructor for the ZeroForceConstraint.
   * @param referenceManager : Switched model ReferenceManager to check for contact.
   * @param contactPointIndex : The 3-DOF contact index this constraint applies to.
   * @param info : The centroidal model information.
   */
  ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex, CentroidalModelInfo info);

  ~ZeroForceConstraint() override = default;
  ZeroForceConstraint* clone() const override { return new ZeroForceConstraint(*this); }

  /**
   * @brief Checks if the constraint is active at a given time (i.e., if the foot is in a swing phase).
   * @param time : The time to check.
   * @return True if the constraint is active, false otherwise.
   */
  bool isActive(scalar_t time) const override;

  /**
   * @brief Returns the number of constraints (3 for the 3D contact force).
   */
  size_t getNumConstraints(scalar_t time) const override { return 3; }

  /**
   * @brief Computes the value of the constraint, which is simply the 3D contact force.
   * The solver will attempt to drive this value to zero.
   */
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;

  /**
   * @brief Computes the linear approximation (Jacobian) of the constraint.
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ZeroForceConstraint(const ZeroForceConstraint& other) = default;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;
};

}  // namespace legged_robot
}  // namespace ocs2
