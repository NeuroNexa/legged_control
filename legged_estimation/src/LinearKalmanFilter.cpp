//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace legged {

KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics),
      numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
      dimContacts_(3 * numContacts_),
      // State vector: [base_pos, base_vel, foot_pos_1, ..., foot_pos_n]
      numState_(6 + dimContacts_),
      // Observation vector: [foot_pos, foot_vel, foot_height] for each foot
      numObserve_(2 * dimContacts_ + numContacts_),
      tfListener_(tfBuffer_),
      topicUpdated_(false) {
  // Initialize state and covariance
  xHat_.setZero(numState_);
  p_ = 100. * matrix_t::Identity(numState_, numState_);

  // Initialize Kalman filter matrices
  a_.setIdentity(numState_, numState_);
  b_.setZero(numState_, 3);
  c_.setZero(numObserve_, numState_);
  q_.setIdentity(numState_, numState_);
  r_.setIdentity(numObserve_, numObserve_);

  // Construct the observation matrix C
  matrix_t c1(3, 6), c2(3, 6);
  c1 << matrix3_t::Identity(), matrix3_t::Zero(); // For position observation
  c2 << matrix3_t::Zero(), matrix3_t::Identity(); // For velocity observation
  for (ssize_t i = 0; i < numContacts_; ++i) {
    c_.block<3, 6>(3 * i, 0) = c1;
    c_.block<3, 6>(3 * (numContacts_ + i), 0) = c2;
    c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0; // For foot height observation
  }
  c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

  // Initialize other variables
  ps_.setZero(dimContacts_);
  vs_.setZero(dimContacts_);
  feetHeights_.setZero(numContacts_);
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);
  world2odom_.setRotation(tf2::Quaternion::getIdentity());

  // Subscribe to external odometry topic
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
}

vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) {
  scalar_t dt = period.toSec();

  // --- Prediction Step ---
  // Build the state transition matrix A for a simple kinematic model
  a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
  // Build the control input matrix B
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
  b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();

  // Update process noise covariance Q
  q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
  q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

  // Use Pinocchio to compute foot positions and velocities (kinematic observations)
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t qPino(info_.generalizedCoordinatesNum);
  vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3), rbdState_.segment<3>(info_.generalizedCoordinatesNum));
  vPino.tail(info_.actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
  const auto eePos = eeKinematics_->getPosition(vector_t());
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  // Set noise values based on config and contact state
  matrix_t q = matrix_t::Identity(numState_, numState_);
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;
  matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
  r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
  r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
      r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
  r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
      r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

  // Increase noise for feet not in contact, as their kinematic measurement is less reliable
  for (int i = 0; i < numContacts_; i++) {
    bool isContact = contactFlag_[i];
    scalar_t high_suspect_number(100); // Factor to increase noise for swing feet
    q.block<3, 3>(6 + 3 * i, 6 + 3 * i) *= (isContact ? 1. : high_suspect_number);
    r.block<3, 3>(3 * i, 3 * i) *= (isContact ? 1. : high_suspect_number);
    r.block<3, 3>(dimContacts_ + 3 * i, dimContacts_ + 3 * i) *= (isContact ? 1. : high_suspect_number);
    r(2 * dimContacts_ + i, 2 * dimContacts_ + i) *= (isContact ? 1. : high_suspect_number);

    // Update the observation vectors (ps_, vs_)
    ps_.segment<3>(i * 3) = -eePos[i];
    ps_.segment<3>(i * 3)[2] += footRadius_;
    vs_.segment<3>(i * 3) = -eeVel[i];
  }

  // Get world-frame acceleration from IMU
  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;

  // Form the full observation vector y
  vector_t y(numObserve_);
  y << ps_, vs_, feetHeights_;

  // Predict state: x_hat_priori = A * x_hat_posteriori + B * u
  xHat_ = a_ * xHat_ + b_ * accel;
  // Predict covariance: P_priori = A * P_posteriori * A' + Q
  matrix_t p_minus = a_ * p_ * a_.transpose() + q;

  // --- Correction Step ---
  // Compute innovation and innovation covariance
  matrix_t y_model = c_ * xHat_;
  matrix_t innovation = y - y_model;
  matrix_t s = c_ * p_minus * c_.transpose() + r;

  // Compute Kalman gain: K = P_priori * C' * S^-1
  vector_t s_inv_innovation = s.lu().solve(innovation);
  // Update state estimate: x_hat_posteriori = x_hat_priori + K * innovation
  xHat_ += p_minus * c_.transpose() * s_inv_innovation;

  // Update state covariance: P_posteriori = (I - K * C) * P_priori
  matrix_t s_inv_c = s.lu().solve(c_);
  p_ = (matrix_t::Identity(numState_, numState_) - p_minus * c_.transpose() * s_inv_c) * p_minus;
  p_ = (p_ + p_.transpose()) / 2.0; // Ensure symmetry

  // If we received a message from the external odom topic, update the state
  if (topicUpdated_) {
    updateFromTopic();
    topicUpdated_ = false;
  }

  // Update the linear part of the base class's state vector
  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  // Publish odometry message
  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbdState_;
}

void KalmanFilterEstimate::updateFromTopic() {
  auto* msg = buffer_.readFromRT();

  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
  {
    tf2::Transform odom2sensor;
    try {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse();
  }
  tf2::Transform base2sensor;
  try {
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
  vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.head<3>() = newPos;
  qPino.segment<3>(3) = rbdState_.head<3>();
  qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  xHat_.segment<3>(0) = newPos;
  for (size_t i = 0; i < numContacts_; ++i) {
    xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
    xHat_(6 + i * 3 + 2) -= footRadius_;
    if (contactFlag_[i]) {
      feetHeights_[i] = xHat_(6 + i * 3 + 2);
    }
  }

  auto odom = getOdomMsg();
  odom.header = msg->header;
  odom.child_frame_id = "base";
  publishMsgs(odom);
}

void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}

nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg() {
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
    }
  }
  return odom;
}

void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose) {
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

}  // namespace legged
