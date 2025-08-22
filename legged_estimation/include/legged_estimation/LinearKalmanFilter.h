//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace legged {
using namespace ocs2;

/**
 * @brief 基于线性卡尔曼滤波器的状态估计器
 *
 * 该类继承自StateEstimateBase，实现了通过卡尔曼滤波器来估计机器人的基座位置和线速度。
 * 它融合了IMU数据和足端运动学信息。
 */
class KalmanFilterEstimate : public StateEstimateBase {
 public:
  /**
   * @brief 构造函数
   */
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief 主更新函数
   *
   * 在每个控制周期执行卡尔曼滤波的预测和更新步骤。
   */
  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief 从配置文件加载滤波器参数
   * @param taskFile 任务配置文件路径
   * @param verbose 是否打印详细信息
   */
  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  /**
   * @brief (未使用) 从话题更新
   */
  void updateFromTopic();

  /**
   * @brief (未使用) ROS话题回调函数
   */
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief (未使用) 获取里程计消息
   */
  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_; // 足端高度

  // --- 卡尔曼滤波器配置参数 ---
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;     // IMU过程噪声（位置）
  scalar_t imuProcessNoiseVelocity_ = 0.02;     // IMU过程噪声（速度）
  scalar_t footProcessNoisePosition_ = 0.002;   // 足端运动学过程噪声
  scalar_t footSensorNoisePosition_ = 0.005;    // 足端位置测量噪声
  scalar_t footSensorNoiseVelocity_ = 0.1;      // 足端速度测量噪声
  scalar_t footHeightSensorNoise_ = 0.01;       // 足端高度测量噪声

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_; // 维度信息

  // --- 卡尔曼滤波器核心矩阵 ---
  matrix_t a_;    // 状态转移矩阵
  matrix_t b_;    // 控制输入矩阵 (未使用)
  matrix_t c_;    // 观测矩阵
  matrix_t q_;    // 过程噪声协方差矩阵
  matrix_t p_;    // 状态估计协方差矩阵
  matrix_t r_;    // 测量噪声协方差矩阵
  vector_t xHat_; // 估计的状态向量 [pos, vel]
  vector_t ps_;   // 足端位置
  vector_t vs_;   // 足端速度

  // --- (未使用) 话题相关 ---
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
};

}  // namespace legged
