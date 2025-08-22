//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @class LeggedController
 * @brief The main controller for the legged robot.
 *
 * This class implements the `controller_interface::MultiInterfaceController` and serves as the central hub
 * for the entire control pipeline. It initializes and manages all the major components, including:
 * - State Estimation: Estimates the robot's current state from sensor data.
 * - Legged Interface: Sets up the optimal control problem for the MPC.
 * - MPC (Model Predictive Control): Computes an optimal state and input trajectory over a future horizon.
 * - WBC (Whole Body Control): Calculates the joint torques required to track the MPC's trajectory.
 *
 * The controller runs two main loops:
 * 1. The `update` loop (real-time): Runs at the frequency of the controller manager. It performs state estimation,
 *    runs the WBC, and sends commands to the hardware.
 * 2. The MPC loop (non-real-time): Runs in a separate thread. It continuously solves the optimal control problem
 *    to provide the latest reference trajectory to the real-time loop.
 */
class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  /**
   * @brief Initializes the controller.
   * @param robot_hw : A pointer to the robot's hardware interface.
   * @param controller_nh : A NodeHandle for the controller's namespace.
   * @return True if initialization is successful.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;

  /**
   * @brief The real-time update loop.
   * @param time : The current time.
   * @param period : The time elapsed since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Called when the controller is started.
   * @param time : The time at which the controller is started.
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief Called when the controller is stopped.
   */
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  /**
   * @brief Updates the state estimate from sensor data.
   */
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  // Setup methods for the main components
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt(); // Model-Reference Tracking (MRT) for communication with the MPC thread
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Hardware and OCS2 Interfaces
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_; // Full rigid body state vector
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

 private:
  // MPC thread management
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};

  // Timers for benchmarking
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

/**
 * @class LeggedCheaterController
 * @brief A derived controller that uses ground truth state information.
 *
 * This "cheater" controller is used for debugging and development, typically in simulation.
 * It bypasses the state estimation module and instead gets perfect state information
 * directly from the simulator (e.g., Gazebo).
 */
class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
