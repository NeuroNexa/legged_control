//
// Created by qiayuan on 23-1-29.
//

#pragma once

#include <ocs2_self_collision/SelfCollisionConstraint.h>

#include "legged_interface/LeggedRobotPreComputation.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @class LeggedSelfCollisionConstraint
 * @brief A specialization of the OCS2 SelfCollisionConstraint for legged robots.
 *
 * This class inherits from the general self-collision constraint provided by ocs2_self_collision.
 * Its primary purpose is to correctly retrieve the Pinocchio interface from the specialized
 * LeggedRobotPreComputation module. This ensures that the collision checking is performed
 * with the robot model that has been updated with the latest state information during the MPC solve.
 */
class LeggedSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  /**
   * @brief Constructor for the LeggedSelfCollisionConstraint.
   * @param mapping : The mapping between the centroidal model and the full Pinocchio model.
   * @param pinocchioGeometryInterface : The interface to the geometry model used for collision checking.
   * @param minimumDistance : The minimum allowable distance between collision pairs.
   */
  LeggedSelfCollisionConstraint(const CentroidalModelPinocchioMapping& mapping, PinocchioGeometryInterface pinocchioGeometryInterface,
                                scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}

  ~LeggedSelfCollisionConstraint() override = default;
  LeggedSelfCollisionConstraint(const LeggedSelfCollisionConstraint& other) = default;
  LeggedSelfCollisionConstraint* clone() const override { return new LeggedSelfCollisionConstraint(*this); }

 protected:
  /**
   * @brief Overrides the base class method to get the Pinocchio interface from the correct pre-computation object.
   * @param preComputation : The pre-computation module from the current MPC time step.
   * @return A constant reference to the PinocchioInterface.
   */
  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    // Cast the generic PreComputation object to the legged-robot-specific type to access its methods.
    return cast<LeggedRobotPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace legged
