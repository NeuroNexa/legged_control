
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

// Conditional compilation to support different versions of the Unitree SDK.
// This allows the code to be compiled against multiple SDKs by defining the correct macro.
#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/safety.h"
#include "unitree_legged_sdk_3_3_1/udp.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/safety.h"
#include "unitree_legged_sdk_3_8_0/udp.h"
#endif

namespace legged {
// Names for the contact sensors, corresponding to the foot names.
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

/**
 * @struct UnitreeMotorData
 * @brief A data structure to hold the state and command for a single Unitree motor.
 */
struct UnitreeMotorData {
  double pos_{}, vel_{}, tau_{};          // Current state: position, velocity, torque
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};  // Command: desired position, velocity, gains, and feed-forward torque
};

/**
 * @struct UnitreeImuData
 * @brief A data structure to hold the data from the Unitree IMU.
 */
struct UnitreeImuData {
  double ori_[4]{};
  double oriCov_[9]{};
  double angularVel_[3]{};
  double angularVelCov_[9]{};
  double linearAcc_[3]{};
  double linearAccCov_[9]{};
};

/**
 * @class UnitreeHW
 * @brief A concrete hardware abstraction layer for Unitree robots (A1, Go1, etc.).
 *
 * This class inherits from `LeggedHW` and implements the `read()` and `write()` methods
 * to communicate with the robot's hardware using the Unitree Legged SDK. It handles
 * sending low-level commands and receiving state data over UDP.
 */
class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;

  /**
   * @brief Initializes the Unitree hardware interface.
   * This function sets up the UDP communication, initializes the joints, IMU, and contact sensors,
   * and registers all the necessary `ros_control` interfaces.
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for the robot hardware namespace.
   * @return True if initialization is successful, false otherwise.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * @brief Reads the latest state data from the robot.
   * This method receives a low-level state packet from the robot via UDP and populates
   * the data buffers for joint states, IMU, and contact sensors.
   * @param time The current time.
   * @param period The time elapsed since the last update.
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Writes the latest commands to the robot.
   * This method populates a low-level command packet with the desired joint commands
   * (position, velocity, kp, kd, feed-forward torque) and sends it to the robot via UDP.
   * @param time The current time.
   * @param period The time elapsed since the last update.
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  /** @brief Initializes the joint handles and registers them with the `ros_control` interfaces. */
  bool setupJoints();
  /** @brief Initializes the IMU handle and registers it with the `ros_control` interfaces. */
  bool setupImu();
  /** @brief Initializes the contact sensor handles and registers them with the `ros_control` interfaces. */
  bool setupContactSensor(ros::NodeHandle& nh);

  // Unitree SDK specific objects
  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{}; // Stores the received state from the robot
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};     // Stores the commands to be sent to the robot

  // Buffers for hardware interface data
  UnitreeMotorData jointData_[12]{};
  UnitreeImuData imuData_{};
  bool contactState_[4]{};

  // Parameters
  int powerLimit_{};
  int contactThreshold_{};
};

}  // namespace legged
