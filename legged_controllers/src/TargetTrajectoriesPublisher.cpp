//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

// --- 全局变量 ---
// 这些变量从配置文件中加载，用于定义目标轨迹的行为。
namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY; // 目标移动速度 (m/s)
scalar_t TARGET_ROTATION_VELOCITY;     // 目标旋转速度 (rad/s)
scalar_t COM_HEIGHT;                   // 期望的质心高度 (m)
vector_t DEFAULT_JOINT_STATE(12);      // 默认的关节位置（站立姿态）
scalar_t TIME_TO_TARGET;               // 对于速度指令，生成的目标轨迹的时域长度
}  // namespace

/**
 * @brief 估算到达目标点所需的时间
 * @param desiredBaseDisplacement 基座需要移动的位移（包括线性和角度）
 * @return 估算的时间 (s)
 */
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY; // 计算旋转所需时间
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY; // 计算平移所需时间
  return std::max(rotationTime, displacementTime); // 取两者中的最大值，确保有足够时间完成动作
}

/**
 * @brief 将一个目标姿态转换为一个两点（当前姿态 -> 目标姿态）的轨迹
 * @param targetPose 目标基座姿态 [x, y, z, yaw, pitch, roll]
 * @param observation 当前的系统观测值
 * @param targetReachingTime 到达目标姿态的绝对时间戳
 * @return 生成的目标轨迹
 */
TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // 期望的时间轨迹 (2个时间点)
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // 期望的状态轨迹 (2个状态点)
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT; // 修正当前z, pitch, roll为默认值
  currentPose(4) = 0;
  currentPose(5) = 0;

  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  // 轨迹点1: 当前状态
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  // 轨迹点2: 目标状态
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // 期望的输入轨迹 (维度正确即可，MPC中通常不直接跟踪输入轨迹)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * @brief 将来自/move_base_simple/goal的指令转换为目标轨迹
 * @param goal 目标指令 [x, y, z, yaw, pitch, roll]
 * @param observation 当前系统观测
 * @return 生成的目标轨迹
 */
TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT; // z, pitch, roll 使用预设值
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  // 估算到达目标所需时间，并计算绝对时间戳
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

/**
 * @brief 将来自/cmd_vel的指令转换为目标轨迹
 * @param cmdVel 速度指令 [vx, vy, vz, wyaw]
 * @param observation 当前系统观测
 * @return 生成的目标轨迹
 */
TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  // 将速度指令从世界坐标系转换到机器人当前的基座坐标系
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  // 预测一小段时间后的目标姿态
  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // 计算目标绝对时间戳
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);

  // 在生成的轨迹中，将质心速度设置为指令速度，使机器人立即开始以该速度移动
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // 初始化ROS节点
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // 从参数服务器加载配置文件
  std::string referenceFile, taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  // 加载全局参数
  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  // 创建并初始化TargetTrajectoriesPublisher对象
  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  // 进入ROS事件循环
  ros::spin();

  return 0; // 成功退出
}
