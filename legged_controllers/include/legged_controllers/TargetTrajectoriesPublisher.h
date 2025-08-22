//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
using namespace ocs2;

/**
 * @class TargetTrajectoriesPublisher
 * @brief A ROS node that listens to high-level commands and publishes OCS2 TargetTrajectories.
 *
 * This class serves as a command interface for the legged robot. It subscribes to two common
 * ROS command topics:
 * 1. `/move_base_simple/goal` (geometry_msgs/PoseStamped): Typically used for navigation goals from RViz.
 * 2. `/cmd_vel` (geometry_msgs/Twist): A standard topic for velocity commands, often used with joysticks.
 *
 * It also subscribes to the MPC's observation topic to get the robot's current state. When a command
 * is received, it uses a provided function to convert that command and the current state into a
 * `TargetTrajectories` message, which is then published for the MPC to use as a reference.
 */
class TargetTrajectoriesPublisher final {
 public:
  /**
   * @brief A function type that defines the conversion from a command vector and a system observation
   * to a TargetTrajectories object.
   */
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  /**
   * @brief Constructor for the TargetTrajectoriesPublisher.
   * @param nh : The ROS NodeHandle.
   * @param topicPrefix : The prefix for the topics to subscribe and publish to.
   * @param goalToTargetTrajectories : A function to convert a goal pose command to target trajectories.
   * @param cmdVelToTargetTrajectories : A function to convert a velocity command to target trajectories.
   */
  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) {
    // Publisher for the target trajectories that the MPC will follow.
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // Subscriber to the MPC observation topic to get the current robot state.
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // Subscriber for goal commands (e.g., from RViz "2D Nav Goal").
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) { // Don't process goals until we have a valid observation.
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      // Transform the goal pose to the odom frame.
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure to transform goal pose: %s\n", ex.what());
        return;
      }

      // Convert the PoseStamped message to a command vector.
      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      const auto eulerAngles = q.toRotationMatrix().eulerAngles(0, 1, 2); // RPY
      cmdGoal[3] = eulerAngles.z(); // Yaw
      cmdGoal[4] = eulerAngles.y(); // Pitch
      cmdGoal[5] = eulerAngles.x(); // Roll

      // Generate and publish the target trajectories.
      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // Subscriber for velocity commands.
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) { // Don't process commands until we have a valid observation.
        return;
      }

      // Convert the Twist message to a command vector.
      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z; // This is often unused for planar robots but included for completeness.
      cmdVel[3] = msg->angular.z;

      // Generate and publish the target trajectories.
      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
  }

 private:
  // Functions to convert commands to trajectories.
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  // ROS publisher.
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  // ROS subscribers and TF listener.
  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  // Thread-safe storage for the latest observation from the MPC.
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
