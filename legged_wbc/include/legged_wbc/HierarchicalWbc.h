//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

/**
 * @class HierarchicalWbc
 * @brief An implementation of the WBC that solves the control problem using a hierarchy of tasks.
 *
 * This class inherits from WbcBase and implements the `update` method. Inside the `update` method,
 * it formulates the control tasks and solves them in a prioritized manner using a cascade of
 * Quadratic Programs (QPs). The solution to a higher-priority QP becomes a constraint for the
 * subsequent lower-priority QPs.
 *
 * The actual implementation of the hierarchical optimization is handled by the HoQp class.
 */
class HierarchicalWbc : public WbcBase {
 public:
  // Inherit the constructor from the base class.
  using WbcBase::WbcBase;

  /**
   * @brief Solves the hierarchical whole-body control problem.
   *
   * This method overrides the base class's update method. It formulates the tasks and then
   * solves them sequentially to find the optimal decision variables.
   *
   * @param stateDesired : The desired state from the MPC.
   * @param inputDesired : The desired input from the MPC.
   * @param rbdStateMeasured : The measured robot state.
   * @param mode : The current contact mode.
   * @param period : The control period.
   * @return The computed optimal decision variables (accelerations, forces, torques).
   */
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
};

}  // namespace legged
