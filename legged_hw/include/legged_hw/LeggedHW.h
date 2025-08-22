
//
// Created by qiayuan on 6/24/22.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {

/**
 * @class LeggedHW
 * @brief The hardware abstraction layer (HAL) for the real legged robot.
 *
 * This class implements the `hardware_interface::RobotHW` from `ros_control`. It serves as the bridge
 * between the high-level controllers and the physical hardware (or a low-level SDK). It is responsible
 * for initializing and registering all the necessary hardware interfaces that the controllers will use
 * to read sensor data (like joint states, IMU data, contact sensors) and send commands (like position,
 * velocity, and torque to the joints).
 *
 * The `read()` and `write()` methods, which handle the communication with the hardware, are typically
 * implemented in the corresponding .cpp file.
 */
class LeggedHW : public hardware_interface::RobotHW {
 public:
  LeggedHW() = default;

  /**
   * @brief Initializes the hardware abstraction layer.
   *
   * This function gets necessary parameters from the ROS parameter server, loads the robot's URDF model,
   * and sets up the hardware interfaces for joints, IMU, and contact sensors.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for the robot hardware namespace.
   * @return True if initialization is successful, false otherwise.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

 protected:
  // ROS Control Interfaces
  hardware_interface::JointStateInterface jointStateInterface_;  //!< For reading joint states (position, velocity, effort).
  hardware_interface::ImuSensorInterface imuSensorInterface_;    //!< For reading IMU data (orientation, angular velocity, linear acceleration).
  HybridJointInterface hybridJointInterface_;                    //!< A custom interface for sending position, velocity, and torque commands simultaneously.
  ContactSensorInterface contactSensorInterface_;                //!< A custom interface for reading contact sensor data.

  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;

 private:
  /**
   * @brief Loads the robot's URDF model from the parameter server.
   * @param rootNh Root node-handle of a ROS node.
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& rootNh);
};

}  // namespace legged
