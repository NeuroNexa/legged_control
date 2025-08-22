//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.hh>
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
 * @brief Abstract base class for state estimators.
 *
 * This class defines the common interface for all state estimation algorithms used in the system.
 * It provides methods to update the estimator with raw sensor data (from joints, contacts, and IMU)
 * and a pure virtual `update` method that derived classes must implement to perform the actual state
 * estimation calculation (e.g., using a Kalman filter).
 */
class StateEstimateBase {
 public:
  /**
   * @brief Constructor for StateEstimateBase.
   * @param pinocchioInterface : A Pinocchio interface for the robot model.
   * @param info : The centroidal model information.
   * @param eeKinematics : The end-effector kinematics.
   */
  StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /** @brief Updates the estimator with new joint position and velocity data. */
  virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);

  /** @brief Updates the estimator with new contact flag data. */
  virtual void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }

  /** @brief Updates the estimator with new IMU data. */
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);

  /**
   * @brief Performs the state estimation update calculation. This is a pure virtual function.
   * @param time : The current time.
   * @param period : The time elapsed since the last update.
   * @return The estimated full rigid body state vector.
   */
  virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;

  /** @brief Gets the current contact mode based on the contact flags. */
  size_t getMode() { return stanceLeg2ModeNumber(contactFlag_); }

 protected:
  /** @brief Updates the angular part of the state from an estimate. */
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
  /** @brief Updates the linear part of the state from an estimate. */
  void updateLinear(const vector_t& pos, const vector_t& linearVel);
  /** @brief Publishes the estimated odometry and pose as ROS messages. */
  void publishMsgs(const nav_msgs::Odometry& odom);

  // Robot model interfaces
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  // Internal state and sensor data storage
  vector3_t zyxOffset_ = vector3_t::Zero();
  vector_t rbdState_; // The full rigid body state being estimated
  contact_flag_t contactFlag_{};
  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  // ROS publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_;
};

/** @brief A simple square function template. */
template <typename T>
T square(T a) {
  return a * a;
}

/**
 * @brief Converts a quaternion to ZYX Euler angles.
 * @param q The input quaternion.
 * @return A 3x1 vector containing the ZYX Euler angles.
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
