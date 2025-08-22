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
#include <ocs2_core/initialization/Initializer.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class LeggedRobotInitializer
 * @brief Provides an initial guess for the state and input trajectories for the MPC solver.
 *
 * In OCS2, the Initializer is responsible for generating a "warm start" for the optimization problem.
 * A good initial guess can significantly reduce the number of iterations required for the solver to converge.
 *
 * This implementation uses the target trajectories from the SwitchedModelReferenceManager as the initial guess.
 * It also handles the initialization of the normalized momentum, which can either be set to zero or
 * extrapolated from the reference trajectory.
 */
class LeggedRobotInitializer final : public Initializer {
 public:
  /**
   * @brief Constructor for LeggedRobotInitializer.
   * @param info : The centroidal model information.
   * @param referenceManager : The reference manager that provides the target trajectories.
   * @param extendNormalizedMomentum : If true, it extrapolates the normalized momenta; otherwise, it sets them to zero.
   */
  LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                         bool extendNormalizedMomentum = false);

  ~LeggedRobotInitializer() override = default;
  LeggedRobotInitializer* clone() const override;

  /**
   * @brief Computes the initial guess for the input and the next state.
   * @param time : The current time.
   * @param state : The current state.
   * @param nextTime : The time of the next step.
   * @param[out] input : The computed initial input.
   * @param[out] nextState : The computed initial next state.
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  LeggedRobotInitializer(const LeggedRobotInitializer& other) = default;

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const bool extendNormalizedMomentum_;
};

}  // namespace legged_robot
}  // namespace ocs2
