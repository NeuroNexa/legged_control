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

#include "legged_interface/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t numConstraints, Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      numConstraints_(numConstraints),
      config_(std::move(config)) {
  // This constraint is designed to work with a single end-effector.
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Standard copy constructor
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs)
    : StateInputConstraint(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorLinearConstraint::configure(Config&& config) {
  // Assert that the provided configuration has the correct dimensions.
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
  config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLinearConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  // Compute the constraint value: g = A_x * x_ee + A_v * v_ee + b
  vector_t f = config_.b;
  if (config_.Ax.size() > 0) {
    // Get end-effector position and add the position term
    f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
  }
  if (config_.Av.size() > 0) {
    // Get end-effector velocity and add the velocity term
    f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front();
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  // Initialize the linear approximation structure.
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  // The constant term is b.
  linearApproximation.f = config_.b;

  // Add the position term contribution, if it exists.
  if (config_.Ax.size() > 0) {
    // Get the linear approximation of the end-effector position.
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    // Add to the constraint value: A_x * x_ee
    linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
    // Add to the state derivative using the chain rule: d(g)/dx = A_x * d(x_ee)/dx
    linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
  }

  // Add the velocity term contribution, if it exists.
  if (config_.Av.size() > 0) {
    // Get the linear approximation of the end-effector velocity.
    const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
    // Add to the constraint value: A_v * v_ee
    linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
    // Add to the state derivative using the chain rule: d(g)/dx = A_v * d(v_ee)/dx
    linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
    // Add to the input derivative using the chain rule: d(g)/du = A_v * d(v_ee)/du
    linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
