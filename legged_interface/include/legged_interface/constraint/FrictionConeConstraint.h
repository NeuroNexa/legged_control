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
#include <ocs2_legged_robot/common/Types.h>
#include "legged_interface/SwitchedModelReferenceManager.h"

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class FrictionConeConstraint
 * @brief Implements the friction cone constraint as a soft or hard inequality constraint.
 *
 * The constraint is formulated as: h(t,x,u) >= 0
 * The specific formula is:
 *   frictionCoefficient * (F_n + gripperForce) - sqrt(F_t1^2 + F_t2^2 + regularization) >= 0
 * where F_n is the normal force and F_t1, F_t2 are the tangential forces in the terrain frame.
 *
 * - The gripperForce allows the model to account for adhesion, enabling tangential forces even with zero normal force,
 *   or even "pulling" forces.
 * - The regularization term prevents singularities in the gradient and Hessian when tangential forces are zero.
 *   It also creates a small safety margin.
 */
class FrictionConeConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @struct Config
   * @brief Configuration parameters for the friction cone constraint.
   */
  struct Config {
    /**
     * @brief Constructor for the Config struct.
     * @param frictionCoefficientParam : The coefficient of friction (mu).
     * @param regularizationParam : A small positive value to regularize the constraint.
     * @param gripperForceParam : The adhesive force available at the contact point.
     * @param hessianDiagonalShiftParam : A shift to ensure the Hessian is positive definite.
     */
    explicit Config(scalar_t frictionCoefficientParam = 0.7, scalar_t regularizationParam = 25.0, scalar_t gripperForceParam = 0.0,
                    scalar_t hessianDiagonalShiftParam = 1e-6)
        : frictionCoefficient(frictionCoefficientParam),
          regularization(regularizationParam),
          gripperForce(gripperForceParam),
          hessianDiagonalShift(hessianDiagonalShiftParam) {
      assert(frictionCoefficient > 0.0);
      assert(regularization > 0.0);
      assert(hessianDiagonalShift >= 0.0);
    }

    scalar_t frictionCoefficient;   //!< The coefficient of friction.
    scalar_t regularization;        //!< A positive number to regularize the friction constraint.
    scalar_t gripperForce;          //!< Gripper force in the normal direction.
    scalar_t hessianDiagonalShift;  //!< A Hessian shift to ensure a strictly-convex quadratic constraint approximation.
  };

  /**
   * @brief Constructor for the FrictionConeConstraint.
   * @param referenceManager : Switched model ReferenceManager to check for contact.
   * @param config : Friction model settings.
   * @param contactPointIndex : The 3-DOF contact index this constraint applies to.
   * @param info : The centroidal model information.
   */
  FrictionConeConstraint(const SwitchedModelReferenceManager& referenceManager, Config config, size_t contactPointIndex,
                         CentroidalModelInfo info);

  ~FrictionConeConstraint() override = default;
  FrictionConeConstraint* clone() const override { return new FrictionConeConstraint(*this); }

  /**
   * @brief Checks if the constraint is active at a given time (i.e., if the foot is in contact).
   * @param time : The time to check.
   * @return True if the constraint is active, false otherwise.
   */
  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; };
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

  /**
   * @brief Sets the estimated terrain normal, expressed in the world frame.
   * This is used to rotate forces into the terrain frame.
   * @param surfaceNormalInWorld : The surface normal vector.
   */
  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

 private:
  // Helper struct for storing derivatives of local forces w.r.t. world frame forces.
  struct LocalForceDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    matrix3_t dF_du;  // derivative local force w.r.t. forces in world frame
  };

  // Helper struct for storing derivatives of the cone constraint w.r.t. local forces.
  struct ConeLocalDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_dF;    // derivative w.r.t local force
    matrix3_t d2Cone_dF2;  // second derivative w.r.t local force
  };

  // Helper struct for storing derivatives of the cone constraint w.r.t. the input vector.
  struct ConeDerivatives {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t dCone_du;
    matrix3_t d2Cone_du2;
  };

  FrictionConeConstraint(const FrictionConeConstraint& other) = default;

  // Computes the value of the friction cone constraint for a given set of local forces.
  vector_t coneConstraint(const vector3_t& localForces) const;

  // Computes derivatives needed for the chain rule.
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives,
                                                   const LocalForceDerivatives& localForceDerivatives) const;

  // Computes the full derivative matrices for the solver.
  matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const;

  const SwitchedModelReferenceManager* referenceManagerPtr_;

  const Config config_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;

  // Rotation from world frame to terrain frame.
  matrix3_t t_R_w = matrix3_t::Identity();
};

}  // namespace legged_robot
}  // namespace ocs2
