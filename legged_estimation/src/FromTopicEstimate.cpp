//
// Created by qiayuan on 2022/7/24.
//

// 修正头文件路径
#include "legged_estimation/FromTopicEstimate.h"

namespace legged {

/**
 * @brief FromTopicStateEstimate 构造函数
 *
 * 初始化基类，并订阅用于提供“真值”状态的ROS话题。
 * @param pinocchioInterface Pinocchio库接口
 * @param info 质心模型信息
 * @param eeKinematics 末端执行器运动学
 */
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
  ros::NodeHandle nh;
  // 订阅 ground_truth/state 话题，通常由Gazebo等仿真器发布
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

/**
 * @brief ROS话题回调函数
 *
 * 当接收到新的里程计消息时，将其写入线程安全的实时缓冲区。
 * @param msg 接收到的 nav_msgs::Odometry 消息指针
 */
void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

/**
 * @brief 主更新函数
 *
 * 从实时缓冲区读取最新的里程计消息，并用其数据更新基座的线性和角度状态。
 * @param time 当前时间 (未使用)
 * @param period 控制周期 (未使用)
 * @return 更新后的RBD状态向量
 */
vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // 从缓冲区读取最新的里程计数据
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  // 从里程计消息中提取位姿和速度信息，并更新状态
  updateAngular(quatToZyx(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  // 将接收到的里程计消息发布出去，供其他节点（如RViz）使用
  publishMsgs(odom);

  return rbdState_;
}

}  // namespace legged
