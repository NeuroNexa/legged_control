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
 * @brief 目标轨迹发布器
 *
 * 该类负责接收来自外部的指令（例如，来自RViz的2D Nav Goal，或来自手柄的cmd_vel），
 * 将这些指令转换为MPC可以理解和跟踪的目标轨迹（ocs2::TargetTrajectories），
 * 并将这些轨迹发布出去，供MPC节点订阅和使用。
 */
class TargetTrajectoriesPublisher final {
 public:
  // 定义一个函数类型，用于将指令向量和当前观测转换为目标轨迹
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param topicPrefix 话题前缀，用于发布和订阅
   * @param goalToTargetTrajectories 将目标点（PoseStamped）指令转换为轨迹的函数
   * @param cmdVelToTargetTrajectories 将速度（Twist）指令转换为轨迹的函数
   */
  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) {
    // 初始化轨迹发布器
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // 订阅MPC的观测数据（当前状态）
    // 这是必需的，因为生成目标轨迹通常需要知道机器人的当前状态
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // 订阅目标点指令 (来自RViz "2D Nav Goal")
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) { // 确保已收到有效的观测数据
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        // 将目标点的坐标系转换到 "odom" 坐标系
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      // 将PoseStamped消息转换为一个6维的目标向量 [x, y, z, roll, pitch, yaw]
      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z(); // ZYX欧拉角 -> yaw
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y(); // pitch
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x(); // roll

      // 调用外部传入的函数，将目标向量转换为目标轨迹
      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      // 发布目标轨迹
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // 订阅速度指令 (来自手柄等)
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      // 将Twist消息转换为一个4维的速度指令向量 [vx, vy, vz, wyaw]
      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      // 调用外部传入的函数，将速度指令转换为目标轨迹
      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      // 发布目标轨迹
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
  }

 private:
  // 指令到轨迹的转换函数
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  // 轨迹发布器
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  // ROS 订阅器和 TF 监听器
  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  // 用于线程安全地访问最新的观测数据
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
