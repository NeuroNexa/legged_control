#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-non-private-member-variables-in-classes"
//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief The LeggedInterface class.
 * It inherits from the ocs2::RobotInterface class and is responsible for setting up the entire optimal control problem for a legged robot.
 * This includes defining the robot model, constraints, cost functions, and solver settings.
 * It serves as a bridge between the high-level controller and the ocs2 library.
 */
class LeggedInterface : public RobotInterface {
 public:
  /**
   * @brief Constructor for the LeggedInterface class.
   * @param taskFile Path to the task configuration file.
   * @param urdfFile Path to the URDF file describing the robot.
   * @param referenceFile Path to the reference configuration file.
   * @param useHardFrictionConeConstraint Whether to use hard or soft friction cone constraints.
   */
  LeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = false);

  ~LeggedInterface() override = default;

  /**
   * @brief Sets up the optimal control problem.
   * This function initializes the robot model, reference manager, pre-computation, and formulates the cost and constraint terms
   * based on the provided configuration files.
   * @param taskFile Path to the task configuration file.
   * @param urdfFile Path to the URDF file.
   * @param referenceFile Path to the reference configuration file.
   * @param verbose Whether to print verbose output during setup.
   */
  virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose);

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }
  const ipm::Settings& ipmSettings() { return ipmSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  PinocchioGeometryInterface& getGeometryInterface() { return *geometryInterfacePtr_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

 protected:
  /**
   * @brief Sets up the robot model using Pinocchio.
   * @param taskFile Path to the task file.
   * @param urdfFile Path to the URDF file.
   * @param referenceFile Path to the reference file.
   * @param verbose Whether to print verbose info.
   */
  virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  /**
   * @brief Sets up the reference manager for trajectory tracking.
   * @param taskFile Path to the task file.
   * @param urdfFile Path to the URDF file.
   * @param referenceFile Path to the reference file.
   * @param verbose Whether to print verbose info.
   */
  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);

  /**
   * @brief Sets up the pre-computation module.
   * @param taskFile Path to the task file.
   * @param urdfFile Path to the URDF file.
   * @param referenceFile Path to the reference file.
   * @param verbose Whether to print verbose info.
   */
  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                   bool verbose);

  /**
   * @brief Loads the gait schedule from a file.
   * @param file Path to the gait file.
   * @param verbose Whether to print verbose info.
   * @return A shared pointer to the GaitSchedule.
   */
  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  /**
   * @brief Creates the cost term for tracking the base state.
   * @param taskFile Path to the task file.
   * @param info The centroidal model info.
   * @param verbose Whether to print verbose info.
   * @return A unique pointer to the StateInputCost for base tracking.
   */
  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);

  /**
   * @brief Initializes the weight matrix for the input cost.
   * @param taskFile Path to the task file.
   * @param info The centroidal model info.
   * @return The input cost weight matrix.
   */
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  /**
   * @brief Loads friction cone settings from a file.
   * @param taskFile Path to the task file.
   * @param verbose Whether to print verbose info.
   * @return A pair containing the friction coefficient and penalty configuration.
   */
  static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose);

  /**
   * @brief Creates a hard friction cone constraint.
   * @param contactPointIndex Index of the contact point (foot).
   * @param frictionCoefficient The friction coefficient.
   * @return A unique pointer to the StateInputConstraint.
   */
  std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);

  /**
   * @brief Creates a soft friction cone constraint (implemented as a cost).
   * @param contactPointIndex Index of the contact point (foot).
   * @param frictionCoefficient The friction coefficient.
   * @param barrierPenaltyConfig Configuration for the relaxed barrier penalty.
   * @return A unique pointer to the StateInputCost.
   */
  std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);

  /**
   * @brief Creates the end-effector kinematics module.
   * @param footNames A vector of foot frame names.
   * @param modelName The name of the robot model.
   * @return A unique pointer to the EndEffectorKinematics.
   */
  std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& footNames,
                                                                      const std::string& modelName);

  /**
   * @brief Creates a zero-velocity constraint for a contact point.
   * @param eeKinematics The end-effector kinematics.
   * @param contactPointIndex Index of the contact point (foot).
   * @return A unique pointer to the StateInputConstraint.
   */
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex);

  /**
   * @brief Creates a self-collision avoidance constraint.
   * @param pinocchioInterface The Pinocchio interface for the robot model.
   * @param taskFile Path to the task file.
   * @param prefix Prefix for loading parameters.
   * @param verbose Whether to print verbose info.
   * @return A unique pointer to the StateCost for self-collision.
   */
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& prefix, bool verbose);

  // Robot model and solver settings
  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  ddp::Settings ddpSettings_;
  sqp::Settings sqpSettings_;
  ipm::Settings ipmSettings_;
  const bool useHardFrictionConeConstraint_;

  // ocs2 interfaces
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

  // Optimal control problem definition
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  // ocs2 rollout and initializer
  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged

#pragma clang diagnostic pop
