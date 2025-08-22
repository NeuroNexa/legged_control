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

#include <memory>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class EndEffectorLinearConstraint
 * @brief Defines a generic linear constraint on an end-effector's position (x_ee) and linear velocity (v_ee).
 * This class serves as a building block for more specific end-effector constraints.
 *
 * The constraint is of the form:
 *   g(x_ee, v_ee) = A_x * x_ee + A_v * v_ee + b
 *
 * Special cases:
 * - For a position-only constraint g(x_ee), set A_v to a zero matrix.
 * - For a velocity-only constraint g(v_ee), set A_x to a zero matrix.
 *
 * The derivatives of this constraint are computed using the end-effector kinematics, which are provided
 * as a CppAD code-generated module.
 */
class EndEffectorLinearConstraint final : public StateInputConstraint {
 public:
  /**
   * @struct Config
   * @brief Coefficients of the linear constraint g(x_ee, v_ee) = A_x * x_ee + A_v * v_ee + b.
   */
  struct Config {
    vector_t b;     //!< The constant vector term.
    matrix_t Ax;    //!< The matrix coefficient for the end-effector position.
    matrix_t Av;    //!< The matrix coefficient for the end-effector velocity.
  };

  /**
   * @brief Constructor for the EndEffectorLinearConstraint.
   * @param endEffectorKinematics : The kinematic interface to the target end-effector.
   * @param numConstraints : The number of constraint equations (the dimension of vector g).
   * @param config : The constraint coefficients (A_x, A_v, b).
   */
  EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numConstraints,
                              Config config = Config());

  ~EndEffectorLinearConstraint() override = default;
  EndEffectorLinearConstraint* clone() const override { return new EndEffectorLinearConstraint(*this); }

  /**
   * @brief Sets new constraint coefficients. This allows for dynamically changing the constraint.
   * @param config : The new configuration to apply.
   */
  void configure(Config&& config);

  /** @brief Overload for lvalue config. */
  void configure(const Config& config) { this->configure(Config(config)); }

  /** @brief Gets the underlying end-effector kinematics interface. */
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t numConstraints_;
  Config config_;
};

}  // namespace legged_robot
}  // namespace ocs2
