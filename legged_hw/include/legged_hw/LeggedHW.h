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
 * @brief 腿式机器人硬件接口的基类
 *
 * 继承自 `hardware_interface::RobotHW`，这是 `ros_control` 框架中所有硬件接口的基类。
 * 它定义了一个通用的腿式机器人硬件接口，聚合了所有必需的子接口（关节、IMU、接触等），
 * 并提供了通用的初始化流程（如加载URDF）。
 * 具体的机器人（如Unitree A1）硬件接口需要继承这个类并实现其 `read` 和 `write` 方法。
 */
class LeggedHW : public hardware_interface::RobotHW {
 public:
  LeggedHW() = default;
  /**
   * @brief 初始化硬件接口
   *
   * @param root_nh 全局ROS节点句柄
   * @param robot_hw_nh 控制器私有的ROS节点句柄
   * @return 如果初始化成功则为 true
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

 protected:
  // --- 硬件接口 ---
  // 这些接口用于在 `ros_control` 框架中注册和管理硬件资源。
  // 派生类需要通过这些接口来注册它们的硬件句柄。
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  HybridJointInterface hybridJointInterface_;
  ContactSensorInterface contactSensorInterface_;

  // 机器人的URDF模型
  std::shared_ptr<urdf::Model> urdfModel_;

 private:
  /**
   * @brief 从参数服务器加载URDF模型
   *
   * @param rootNh 全局ROS节点句柄
   * @return 如果加载成功则为 true
   */
  bool loadUrdf(ros::NodeHandle& rootNh);
};

}  // namespace legged
