//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/FromTopiceEstimate.h"

namespace legged {
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
  ros::NodeHandle nh;
  // Subscribe to the ground truth state topic.
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Callback function to write the received message into a real-time buffer.
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Read the latest odometry message from the buffer.
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  // Extract the angular and linear components from the odometry message.
  const auto& pose = odom.pose.pose;
  const auto& twist = odom.twist.twist;
  Eigen::Quaternion<scalar_t> quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  vector3_t angularVel(twist.angular.x, twist.angular.y, twist.angular.z);
  vector3_t pos(pose.position.x, pose.position.y, pose.position.z);
  vector3_t linearVel(twist.linear.x, twist.linear.y, twist.linear.z);

  // Update the base class's state vector with the data from the topic.
  updateAngular(quatToZyx(quat), angularVel);
  updateLinear(pos, linearVel);

  // Publish the odometry message for other nodes to use.
  publishMsgs(odom);

  // Return the full rigid body state.
  return rbdState_;
}

}  // namespace legged
