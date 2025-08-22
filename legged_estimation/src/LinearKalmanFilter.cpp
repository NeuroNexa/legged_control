//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

/**
 * @brief KalmanFilterEstimate 构造函数
 *
 * 初始化状态估计器，设置卡尔曼滤波器的矩阵维度和初始值。
 */
KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics),
      numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
      dimContacts_(3 * numContacts_),
      numState_(6 + dimContacts_), // 状态量维度: [base_pos(3), base_vel(3), foot_pos(3*4)]
      numObserve_(2 * dimContacts_ + numContacts_), // 观测量维度: [foot_pos_kin(3*4), foot_vel_kin(3*4), foot_height(4)]
      tfListener_(tfBuffer_),
      topicUpdated_(false) {
  // --- 初始化卡尔曼滤波器矩阵 ---
  xHat_.setZero(numState_); // 状态估计向量
  ps_.setZero(dimContacts_); // 足端位置测量
  vs_.setZero(dimContacts_); // 足端速度测量
  a_.setIdentity(numState_, numState_); // 状态转移矩阵 A
  b_.setZero(numState_, 3); // 控制输入矩阵 B

  // 初始化观测矩阵 C
  matrix_t c1(3, 6), c2(3, 6);
  c1 << matrix3_t::Identity(), matrix3_t::Zero();
  c2 << matrix3_t::Zero(), matrix3_t::Identity();
  c_.setZero(numObserve_, numState_);
  for (ssize_t i = 0; i < numContacts_; ++i) {
    c_.block(3 * i, 0, 3, 6) = c1; // 观测足端位置(通过运动学) vs 估计的基座位置
    c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2; // 观测足端速度(通过运动学) vs 估计的基座速度
    c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0; // 观测足端高度 vs 估计的足端高度
  }
  c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

  // 初始化协方差矩阵
  q_.setIdentity(numState_, numState_); // 过程噪声协方差 Q
  p_ = 100. * q_; // 状态估计协方差 P
  r_.setIdentity(numObserve_, numObserve_); // 测量噪声协方差 R
  feetHeights_.setZero(numContacts_);

  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  // (未使用) TF和话题订阅相关
  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
}

/**
 * @brief 主更新函数
 *
 * 执行卡尔曼滤波的预测和更新步骤。
 * @return 更新后的RBD状态向量
 */
vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) {
  scalar_t dt = period.toSec();

  // --- 1. 更新状态转移矩阵 A 和控制输入矩阵 B ---
  // x_{k+1} = A * x_k + B * u_k
  // [pos] = [pos] + dt * [vel] + 0.5 * dt^2 * [accel]
  // [vel] = [vel] + dt * [accel]
  a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
  b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();

  // --- 2. 更新过程噪声协方差矩阵 Q ---
  // 这部分是基于经验的模型不确定性
  q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
  q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

  // --- 3. 计算足端运动学 ---
  // 使用Pinocchio库计算由关节状态得到的足端位置和速度
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t qPino(info_.generalizedCoordinatesNum), vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>(); // 只设置姿态，位置保持原点
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qPino.segment<3>(3), rbdState_.segment<3>(info_.generalizedCoordinatesNum));
  vPino.tail(info_.actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
  const auto eePos = eeKinematics_->getPosition(vector_t());
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  // --- 4. 根据接触状态调整噪声 ---
  // 核心思想：对于接触的脚，我们更相信它的运动学模型（速度应为0），所以减小其过程噪声；
  // 对于摆动的脚，我们不确定它的位置，所以增大其过程噪声。
  matrix_t q = matrix_t::Identity(numState_, numState_);
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;
  matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
  r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
  r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) = r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
  r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) = r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

  for (int i = 0; i < numContacts_; i++) {
    bool isContact = contactFlag_[i];
    scalar_t high_suspect_number(100); // 增大噪声的倍数
    q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(6 + 3 * i, 6 + 3 * i, 3, 3);
    r.block(3 * i, 3 * i, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(3 * i, 3 * i, 3, 3);
    r.block(dimContacts_ + 3 * i, dimContacts_ + 3 * i, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(dimContacts_ + 3 * i, dimContacts_ + 3 * i, 3, 3);
    r(2 * dimContacts_ + i, 2 * dimContacts_ + i) = (isContact ? 1. : high_suspect_number) * r(2 * dimContacts_ + i, 2 * dimContacts_ + i);

    // 填充测量向量中的足端位置和速度
    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];
  }

  // --- 5. 卡尔曼滤波核心步骤 ---
  // a. 预测
  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;
  xHat_ = a_ * xHat_ + b_ * accel; // 状态预测
  p_ = a_ * p_ * a_.transpose() + q; // 协方差预测

  // b. 更新 (测量更新)
  vector_t y(numObserve_);
  y << ps_, vs_, feetHeights_; // 构造完整的测量向量 y
  matrix_t yModel = c_ * xHat_; // 根据预测的状态计算预期的测量值
  matrix_t ey = y - yModel; // 测量残差 (innovation)
  matrix_t s = c_ * p_ * c_.transpose() + r; // 残差协方差
  vector_t sEy = s.lu().solve(ey); // 使用LU分解求解
  xHat_ += p_ * c_.transpose() * sEy; // 状态更新
  p_ = (matrix_t::Identity(numState_, numState_) - p_ * c_.transpose() * s.lu().solve(c_)) * p_; // 协方差更新
  p_ = (p_ + p_.transpose()) / 2.0; // 保持协方差矩阵对称

  // --- 6. 更新并发布状态 ---
  if (topicUpdated_) { // (未使用)
    updateFromTopic();
    topicUpdated_ = false;
  }
  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3)); // 将滤波后的基座状态更新到RBD状态向量
  auto odom = getOdomMsg(); // 构造里程计消息
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbdState_;
}

// ... (以下是未使用或辅助的函数)

void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose) {
  // 从配置文件加载噪声参数
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "kalmanFilter.";
  if (verbose) {
    std::cerr << "\n #### Kalman Filter Noise:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
  loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
  loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
}

// ... (getOdomMsg, callback, updateFromTopic 等函数的实现)
nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg() {
  nav_msgs::Odometry odom;
  // ... (填充里程计消息)
  return odom;
}
void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}
void KalmanFilterEstimate::updateFromTopic() {
  // ...
}

}  // namespace legged
