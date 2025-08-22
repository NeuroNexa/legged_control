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
 * @brief 状态估计的基类
 *
 * 定义了所有状态估计器（如卡尔曼滤波、或从话题订阅）的通用接口。
 * 它处理来自硬件的输入（关节、IMU、接触），并提供一个统一的 `update` 方法来生成最终的状态估计。
 */
class StateEstimateBase {
 public:
  /**
   * @brief 构造函数
   * @param pinocchioInterface Pinocchio库的接口，用于运动学计算
   * @param info 质心模型信息
   * @param eeKinematics 末端执行器运动学
   */
  StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief 更新关节状态
   * @param jointPos 关节位置
   * @param jointVel 关节速度
   */
  virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);

  /**
   * @brief 更新接触状态
   * @param contactFlag 接触标志位
   */
  virtual void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }

  /**
   * @brief 更新IMU数据
   * @param quat 四元数姿态
   * @param angularVelLocal 本体系角速度
   * @param linearAccelLocal 本体系线加速度
   * @param orientationCovariance 姿态协方差
   * @param angularVelCovariance 角速度协方差
   * @param linearAccelCovariance 线加速度协方差
   */
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);

  /**
   * @brief 主更新函数，由派生类实现
   * @param time 当前时间
   * @param period 控制周期
   * @return 估计的机器人状态（RBDL模型格式）
   */
  virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;

  /**
   * @brief 获取步态模式编号
   * @return 步态模式
   */
  size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

 protected:
  /**
   * @brief 更新RBD状态中的角度部分
   * @param zyx 欧拉角 (Z-Y-X)
   * @param angularVel 角速度
   */
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);

  /**
   * @brief 更新RBD状态中的线性部分
   * @param pos 位置
   * @param linearVel 线速度
   */
  void updateLinear(const vector_t& pos, const vector_t& linearVel);

  /**
   * @brief 发布里程计和位姿消息
   * @param odom 里程计消息
   */
  void publishMsgs(const nav_msgs::Odometry& odom);

  // --- 成员变量 ---
  PinocchioInterface pinocchioInterface_; // Pinocchio接口
  CentroidalModelInfo info_; // 质心模型信息
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_; // 末端执行器运动学

  vector3_t zyxOffset_ = vector3_t::Zero(); // 欧拉角偏移量
  vector_t rbdState_; // 机器人状态（RBDL格式）
  contact_flag_t contactFlag_{}; // 接触标志
  Eigen::Quaternion<scalar_t> quat_; // 四元数
  vector3_t angularVelLocal_, linearAccelLocal_; // IMU测量的角速度和线加速度
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_; // IMU协方差

  // ROS发布器
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_; // 上次发布消息的时间
};

/**
 * @brief 计算一个数的平方
 */
template <typename T>
T square(T a) {
  return a * a;
}

/**
 * @brief 将四元数转换为Z-Y-X欧拉角
 * @param q 输入的四元数
 * @return ZYX欧拉角向量
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z())); // Yaw
  zyx(1) = std::asin(as); // Pitch
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z())); // Roll
  return zyx;
}

}  // namespace legged
