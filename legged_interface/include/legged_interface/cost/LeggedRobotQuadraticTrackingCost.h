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
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_legged_robot/common/utils.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class LeggedRobotStateInputQuadraticCost
 * @brief A quadratic cost on state and input, specialized for legged robots.
 *
 * This cost term penalizes the deviation of the state and input from a desired reference.
 * The cost is of the form: 0.5 * [x-x_ref, u-u_ref]' * [Q, P; P', R] * [x-x_ref, u-u_ref]
 *
 * A key feature of this implementation is that it redefines the nominal input `u_ref`. Instead of tracking a
 * zero input, it tracks the input required to compensate for gravity given the current contact configuration.
 * This means the R matrix penalizes inputs that deviate from the gravity-compensating effort, which is
 * often more desirable for legged robots.
 */
class LeggedRobotStateInputQuadraticCost final : public QuadraticStateInputCost {
 public:
  /**
   * @brief Constructor for LeggedRobotStateInputQuadraticCost.
   * @param Q : The state weighting matrix.
   * @param R : The input weighting matrix.
   * @param info : The centroidal model information.
   * @param referenceManager : The reference manager to get contact flags.
   */
  LeggedRobotStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                     const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateInputQuadraticCost() override = default;
  LeggedRobotStateInputQuadraticCost* clone() const override { return new LeggedRobotStateInputQuadraticCost(*this); }

 private:
  LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs) = default;

  /**
   * @brief Computes the deviation of the state and input from their respective references.
   * This override is the core of this class's specialization.
   */
  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    // Get the desired state from the reference trajectory.
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    // Get the current contact flags.
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    // Calculate the nominal input required to compensate for gravity.
    const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
    // Return the deviations from the desired state and the gravity-compensating input.
    return {state - xNominal, input - uNominal};
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

/**
 * @class LeggedRobotStateQuadraticCost
 * @brief A quadratic cost on the final state, specialized for legged robots.
 *
 * This cost term is typically used at the end of the MPC horizon to penalize the
 * deviation of the final state from a desired terminal state.
 */
class LeggedRobotStateQuadraticCost final : public QuadraticStateCost {
 public:
  /**
   * @brief Constructor for LeggedRobotStateQuadraticCost.
   * @param Q : The state weighting matrix.
   * @param info : The centroidal model information.
   * @param referenceManager : The reference manager.
   */
  LeggedRobotStateQuadraticCost(matrix_t Q, CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager)
      : QuadraticStateCost(std::move(Q)), info_(std::move(info)), referenceManagerPtr_(&referenceManager) {}

  ~LeggedRobotStateQuadraticCost() override = default;
  LeggedRobotStateQuadraticCost* clone() const override { return new LeggedRobotStateQuadraticCost(*this); }

 private:
  LeggedRobotStateQuadraticCost(const LeggedRobotStateQuadraticCost& rhs) = default;

  /**
   * @brief Computes the deviation of the state from its reference.
   */
  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const override {
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    return state - xNominal;
  }

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
