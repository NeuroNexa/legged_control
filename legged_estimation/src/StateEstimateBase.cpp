//
// Created by qiayuan on 2021/11/15.
//

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {
using namespace legged_robot;

/**
 * @brief StateEstimateBase 构造函数
 */
StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      eeKinematics_(eeKinematics.clone()),
      rbdState_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)) { // 初始化RBD状态向量，大小为 2 * 广义坐标数 (位置+速度)
  ros::NodeHandle nh;
  // 初始化里程计和位姿的ROS实时发布器
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));
  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));
}

/**
 * @brief 更新关节状态
 *
 * 将测量的关节位置和速度填充到RBD状态向量的相应部分。
 * @param jointPos 测量的关节位置
 * @param jointVel 测量的关节速度
 */
void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
  // rbdState_ 向量的 [6, 6 + N-1] 是关节位置 (N=关节数)
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  // rbdState_ 向量的 [6+generalizedCoordinatesNum, 6+generalizedCoordinatesNum + N-1] 是关节速度
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

/**
 * @brief 更新IMU数据
 *
 * 处理原始IMU数据，计算欧拉角和全局角速度，并调用updateAngular进行更新。
 * @param quat IMU的四元数
 * @param angularVelLocal IMU的局部角速度
 * @param linearAccelLocal IMU的局部线加速度
 * ... (协方差参数，当前未使用)
 */
void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  // 存储原始IMU数据
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  // 将四元数转换为ZYX欧拉角，并减去初始偏移
  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  // 将局部角速度转换为全局坐标系下的欧拉角导数，再转换为全局角速度
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  // 更新RBD状态的角度部分
  updateAngular(zyx, angularVelGlobal);
}

/**
 * @brief 更新RBD状态的角度部分
 * @param zyx ZYX欧拉角
 * @param angularVel 全局角速度
 */
void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
  // rbdState_ 向量的 [0, 2] 是基座的欧拉角
  rbdState_.segment<3>(0) = zyx;
  // rbdState_ 向量的 [generalizedCoordinatesNum, generalizedCoordinatesNum+2] 是基座的角速度
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

/**
 * @brief 更新RBD状态的线性部分
 * @param pos 基座位置
 * @param linearVel 基座线速度
 */
void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  // rbdState_ 向量的 [3, 5] 是基座的位置
  rbdState_.segment<3>(3) = pos;
  // rbdState_ 向量的 [generalizedCoordinatesNum+3, generalizedCoordinatesNum+5] 是基座的线速度
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

/**
 * @brief 发布里程计和位姿消息
 *
 * 使用一个固定的频率来限制消息的发布速率。
 * @param odom 要发布的里程计消息
 */
void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    lastPub_ = time;
    // 使用trylock()来避免在发布时阻塞主控制循环
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

}  // namespace legged
