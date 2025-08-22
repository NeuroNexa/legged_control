//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/StateEstimateBase.h"

#include <realtime_tools/realtime_buffer.h>

#pragma once
namespace legged {
using namespace ocs2;

/**
 * @class FromTopicStateEstimate
 * @brief A "cheater" state estimator that gets the robot's state directly from a ROS topic.
 *
 * This class is a simple implementation of StateEstimateBase that does not perform any sensor fusion
 * or filtering. Instead, it subscribes to a nav_msgs/Odometry topic and uses the received messages
 * as the ground truth state of the robot. This is useful for debugging other parts of the control
 * system in a simulated environment where the ground truth is available.
 */
class FromTopicStateEstimate : public StateEstimateBase {
 public:
  /**
   * @brief Constructor for FromTopicStateEstimate.
   * @param pinocchioInterface : A Pinocchio interface for the robot model.
   * @param info : The centroidal model information.
   * @param eeKinematics : The end-effector kinematics.
   */
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief Overrides the base class method to do nothing, as IMU data is not used by this estimator.
   */
  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  /**
   * @brief "Updates" the state by taking the latest message from the subscribed odometry topic.
   * @param time : The current time.
   * @param period : The time elapsed since the last update.
   * @return The full rigid body state vector from the topic.
   */
  vector_t update(const ros::Time& time, const ros::Duration& period) override;

 private:
  /** @brief ROS subscriber callback to buffer incoming odometry messages. */
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_; // A thread-safe buffer for the latest odometry message.
};

}  // namespace legged
