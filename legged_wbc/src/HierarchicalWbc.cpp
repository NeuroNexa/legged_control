//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  // Update the base class with the latest state information.
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Formulate the tasks and group them by priority.
  // The operator+ for Tasks combines them into a single task.

  // Task 0: Highest priority. This group includes fundamental physical constraints.
  // - Equations of Motion: Must be satisfied.
  // - Torque Limits: Physical limits of the actuators.
  // - Friction Cone: Prevents feet from slipping.
  // - No Contact Motion: Ensures stance feet remain on the ground.
  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();

  // Task 1: Medium priority. This group handles motion tracking.
  // - Base Acceleration: Tracks the desired base motion from the MPC.
  // - Swing Leg: Tracks the desired swing foot trajectories.
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();

  // Task 2: Lowest priority. This group handles desired force tracking.
  // - Contact Force: Tries to achieve the desired contact forces from the MPC. This is often a "soft" objective.
  Task task2 = formulateContactForceTask(inputDesired);

  // Construct the cascade of Quadratic Programs (QPs) using HoQp (Hierarchical Optimization for QP).
  // The QPs are nested, so the solution of the higher-priority QP becomes a constraint for the lower-priority one.
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  // Solve the hierarchical QP and return the solution.
  return hoQp.getSolutions();
}

}  // namespace legged
