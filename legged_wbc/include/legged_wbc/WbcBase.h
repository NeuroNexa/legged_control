//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @class WbcBase
 * @brief Base class for the Whole Body Controller (WBC).
 *
 * The WBC calculates the optimal joint torques and contact forces to achieve a desired motion,
 * while respecting various physical and task-space constraints. It formulates the control problem
 * as a set of prioritized tasks (constraints and objectives) that are then solved by a derived class
 * (e.g., HierarchicalWbc or WeightedWbc).
 *
 * The decision variables for the optimization problem are:
 *   x = [generalized_accelerations^T, contact_forces^T, joint_torques^T]^T
 */
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  /**
   * @brief Constructor for WbcBase.
   * @param pinocchioInterface : A Pinocchio interface for the robot model.
   * @param info : The centroidal model information.
   * @param eeKinematics : The end-effector kinematics.
   */
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief Loads task-specific settings from a configuration file.
   * @param taskFile : Path to the task configuration file.
   * @param verbose : Whether to print loaded settings.
   */
  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  /**
   * @brief The main update loop for the WBC.
   * @param stateDesired : The desired state from the MPC.
   * @param inputDesired : The desired input from the MPC.
   * @param rbdStateMeasured : The measured robot state (generalized coordinates and velocities).
   * @param mode : The current contact mode.
   * @param period : The control period.
   * @return The computed optimal decision variables (accelerations, forces, torques).
   */
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  /**
   * @brief Updates the internal model with the measured robot state.
   * @param rbdStateMeasured : The measured robot state.
   */
  void updateMeasured(const vector_t& rbdStateMeasured);

  /**
   * @brief Updates the internal model with the desired state and input.
   * @param stateDesired : The desired state.
   * @param inputDesired : The desired input.
   */
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  // Task formulation methods
  Task formulateFloatingBaseEomTask();        //!< Formulates the floating base equations of motion task.
  Task formulateTorqueLimitsTask();           //!< Formulates the joint torque limits task.
  Task formulateNoContactMotionTask();        //!< Formulates the task for unconstrained motion of swing feet.
  Task formulateFrictionConeTask();           //!< Formulates the friction cone constraints for stance feet.
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);  //!< Formulates the base acceleration tracking task.
  Task formulateSwingLegTask();               //!< Formulates the swing leg motion tracking task.
  Task formulateContactForceTask(const vector_t& inputDesired) const;  //!< Formulates the contact force tracking task.

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  // Internal state variables updated at each cycle
  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;  // Contact Jacobian and its time derivative
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters loaded from config file
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace legged
