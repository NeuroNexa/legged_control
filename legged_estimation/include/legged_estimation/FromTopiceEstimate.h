//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/StateEstimateBase.h"

#include <realtime_tools/realtime_buffer.h>

#pragma once
namespace legged {
using namespace ocs2;

/**
 * @brief 从话题获取状态的估计器 (作弊用)
 *
 * 该类继承自StateEstimateBase，但它不执行任何实际的估计计算。
 * 相反，它直接订阅一个包含真实状态（通常由Gazebo等仿真器发布）的`nav_msgs/Odometry`话题，
 * 并将其作为状态估计的结果。
 * 这对于在仿真中排除状态估计误差，专注于控制器本身的调试非常有用。
 */
class FromTopicStateEstimate : public StateEstimateBase {
 public:
  /**
   * @brief 构造函数
   */
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief 重载的updateImu函数
   *
   * 由于该类直接获取真值，因此不需要处理IMU数据，此函数为空。
   */
  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  /**
   * @brief 主更新函数
   *
   * 从实时缓冲区中获取最新的里程计消息，并将其转换为RBD状态向量。
   */
  vector_t update(const ros::Time& time, const ros::Duration& period) override;

 private:
  /**
   * @brief ROS话题回调函数
   *
   * 将接收到的里程计消息写入实时缓冲区。
   */
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_; // ROS订阅器
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_; // 实时缓冲区，用于线程安全地存储接收到的消息
};

}  // namespace legged
