//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

/**
 * @class StateEstimateBase
 * @brief 状态估计器的抽象基类。
 *
 * 此类定义了系统中所有状态估计算法的通用接口。
 * 它提供了用原始传感器数据（来自关节、触点和IMU）更新估计器的方法，
 * 以及一个纯虚的 `update` 方法，派生类必须实现该方法以执行实际的状态估计计算
 * （例如，使用卡尔曼滤波器）。
 */
class StateEstimateBase {
 public:
  /**
   * @brief StateEstimateBase 的构造函数。
   * @param pinocchioInterface 机器人的 Pinocchio 接口。
   * @param info 质心模型信息。
   * @param eeKinematics 末端执行器运动学。
   */
  StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
  /** @brief 用新的关节位置和速度数据更新估计器。 */
  virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);
  /** @brief 用新的接触标志数据更新估计器。 */
  virtual void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
  /** @brief 用新的IMU数据更新估计器。 */
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);

  /**
   * @brief 执行状态估计更新计算。这是一个纯虚函数。
   * @param time 当前时间。
   * @param period 自上次更新以来经过的时间。
   * @return 估计的完整刚体状态向量。
   */
  virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;

  /** @brief 根据接触标志获取当前接触模式。 */
  size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

 protected:
  /** @brief 从估计值更新状态的角度部分。 */
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
  /** @brief 从估计值更新状态的线性部分。 */
  void updateLinear(const vector_t& pos, const vector_t& linearVel);
  /** @brief 将估计的里程计和位姿作为ROS消息发布。 */
  void publishMsgs(const nav_msgs::Odometry& odom);

  // 机器人模型接口
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  // 内部状态和传感器数据存储
  vector3_t zyxOffset_ = vector3_t::Zero();
  vector_t rbdState_; // 正在估计的完整刚体状态
  contact_flag_t contactFlag_{};
  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  // ROS 发布器
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_;
};

/** @brief 一个简单的平方函数模板。 */
template <typename T>
T square(T a) {
  return a * a;
}

/**
 * @brief 将四元数转换为 ZYX 欧拉角。
 * @param q 输入的四元数。
 * @return 包含 ZYX 欧拉角的 3x1 向量。
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace legged
