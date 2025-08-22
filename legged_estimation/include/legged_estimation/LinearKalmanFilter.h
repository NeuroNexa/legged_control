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
 * @class KalmanFilterEstimate
 * @brief A state estimator for legged robots using a linear Kalman filter.
 *
 * This class inherits from StateEstimateBase and implements a linear Kalman filter to estimate
 * the robot's base position and linear velocity. It uses an IMU for prediction and the kinematics
 * of feet in contact with the ground for correction.
 *
 * The state vector `x` is: [base_pos, base_vel, foot_pos_in_world_frame_1, ..., foot_pos_in_world_frame_n]^T
 * The observation vector `y` is: [foot_pos_from_imu, foot_vel_from_imu, foot_height_from_imu]^T
 *
 * It can also subscribe to an external odometry topic to get an initial guess for the robot's pose.
 */
class KalmanFilterEstimate : public StateEstimateBase {
 public:
  /**
   * @brief Constructor for KalmanFilterEstimate.
   * @param pinocchioInterface : A Pinocchio interface for the robot model.
   * @param info : The centroidal model information.
   * @param eeKinematics : The end-effector kinematics.
   */
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief Performs the Kalman filter prediction and correction steps.
   * @param time : The current time.
   * @param period : The time elapsed since the last update.
   * @return The estimated full rigid body state vector.
   */
  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Loads Kalman filter settings (e.g., noise parameters) from a configuration file.
   * @param taskFile : Path to the configuration file.
   * @param verbose : Whether to print loaded settings.
   */
  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  /** @brief Updates the filter's state from an external odometry topic. */
  void updateFromTopic();

  /** @brief ROS subscriber callback for the external odometry topic. */
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  /** @brief Constructs an odometry message from the current state estimate. */
  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Kalman Filter Configuration Parameters
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_{}, dimContacts_{}, numState_{}, numObserve_{};

  // Kalman Filter Matrices
  matrix_t a_;  //!< State transition matrix
  matrix_t b_;  //!< Control input matrix
  matrix_t c_;  //!< Observation matrix
  matrix_t q_;  //!< Process noise covariance
  matrix_t p_;  //!< State estimate covariance
  matrix_t r_;  //!< Observation noise covariance

  // Kalman Filter State
  vector_t xHat_;  //!< Estimated state vector [pos, vel, foot_pos_1, ..., foot_pos_n]
  vector_t ps_;    //!< Foot positions in base frame
  vector_t vs_;    //!< Foot velocities in base frame

  // ROS Topic Handling for External Odometry
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_ = false;
};

}  // namespace legged
