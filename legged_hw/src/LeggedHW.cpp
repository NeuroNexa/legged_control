//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/LeggedHW.h"

namespace legged {
/**
 * @brief 初始化硬件接口基类
 *
 * @param root_nh 全局ROS节点句柄
 * @param robot_hw_nh 机器人硬件私有的ROS节点句柄 (未使用)
 * @return 如果初始化成功则为 true
 */
bool LeggedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  // 从参数服务器加载URDF
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  // 向ros_control框架注册该硬件支持的所有接口
  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  return true;
}

/**
 * @brief 从参数服务器加载URDF
 *
 * @param rootNh 全局ROS节点句柄
 * @return 如果加载和解析成功则为 true
 */
bool LeggedHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  // 如果urdfModel_智能指针为空，则为其分配一个新的urdf::Model对象
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // 从参数服务器的 "legged_robot_description" 参数中获取URDF的XML字符串
  rootNh.getParam("legged_robot_description", urdfString);
  // 使用urdf::Model的initString方法来解析URDF字符串
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace legged
