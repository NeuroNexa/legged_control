
//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/LeggedHW.h"

namespace legged {
bool LeggedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  // Load the URDF model from the parameter server.
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up URDF");
    return false;
  }

  // Register the hardware interfaces with the RobotHW class.
  // This makes the interfaces available to the controller manager.
  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  return true;
}

bool LeggedHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // Get the URDF XML string from the parameter server.
  // The parameter name is hard-coded to "legged_robot_description".
  rootNh.getParam("legged_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

// NOTE: The read() and write() methods are not implemented in this base class.
// A robot-specific hardware interface class that inherits from LeggedHW
// would need to implement these methods to communicate with the actual hardware
// (e.g., via CAN bus, EtherCAT, or an SDK).

}  // namespace legged
