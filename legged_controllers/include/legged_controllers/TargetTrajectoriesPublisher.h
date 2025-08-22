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
 * @brief 一个监听高级指令并发布 OCS2 目标轨迹的 ROS 节点。
 *
 * 此类作为足式机器人的指令接口。它订阅两个常见的 ROS 指令话题：
 * 1. `/move_base_simple/goal` (geometry_msgs/PoseStamped): 通常用于 RViz 的导航目标。
 * 2. `/cmd_vel` (geometry_msgs/Twist): 用于速度指令的标准话题，常与手柄配合使用。
 *
 * 它还订阅 MPC 的观测话题以获取机器人的当前状态。当收到指令时，
 * 它使用一个提供的函数将该指令和当前状态转换为 `TargetTrajectories` 消息，
 * 然后发布该消息供 MPC 用作参考。
 */
class TargetTrajectoriesPublisher final {
 public:
  /**
   * @brief 一个函数类型，定义了从指令向量和系统观测到 TargetTrajectories 对象的转换。
   */
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  /**
   * @brief TargetTrajectoriesPublisher 的构造函数。
   * @param nh ROS 节点句柄。
   * @param topicPrefix 用于订阅和发布话题的前缀。
   * @param goalToTargetTrajectories 将目标姿态指令转换为目标轨迹的函数。
   * @param cmdVelToTargetTrajectories 将速度指令转换为目标轨迹的函数。
   */
  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) {
    // MPC 将遵循的目标轨迹的发布者。
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // 订阅 MPC 观测话题以获取当前机器人状态。
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // 目标指令的订阅者（例如，来自 RViz 的 "2D Nav Goal"）。
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) { // 在获得有效观测值之前不处理目标。
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      // 将目标姿态转换到 odom 坐标系。
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("转换目标姿态失败: %s\n", ex.what());
        return;
      }

      // 将 PoseStamped 消息转换为指令向量。
      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      const auto eulerAngles = q.toRotationMatrix().eulerAngles(0, 1, 2); // RPY
      cmdGoal[3] = eulerAngles.z(); // Yaw
      cmdGoal[4] = eulerAngles.y(); // Pitch
      cmdGoal[5] = eulerAngles.x(); // Roll

      // 生成并发布目标轨迹。
      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // 速度指令的订阅者。
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) { // 在获得有效观测值之前不处理指令。
        return;
      }

      // 将 Twist 消息转换为指令向量。
      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z; // 对于平面机器人通常不使用，但为完整性保留。
      cmdVel[3] = msg->angular.z;

      // 生成并发布目标轨迹。
      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
  }

 private:
  // 将指令转换为轨迹的函数。
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  // ROS 发布者。
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  // ROS 订阅者和 TF 监听器。
  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  // 用于 MPC 最新观测值的线程安全存储。
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
