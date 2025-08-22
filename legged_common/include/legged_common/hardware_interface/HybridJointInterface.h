//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {

/**
 * @class HybridJointHandle
 * @brief A handle to a single "hybrid" joint.
 *
 * This class extends the standard `JointStateHandle` to provide an interface for commanding a joint
 * with multiple control modes simultaneously. It allows setting a desired position and velocity,
 * PD gains (Kp and Kd), and a feed-forward torque/effort.
 *
 * This is a common control strategy for legged robots, where a low-level motor controller
 * runs a PD loop, and the high-level controller (like a WBC) provides the setpoints and a
 * feed-forward torque calculated from a dynamics model.
 */
class HybridJointHandle : public hardware_interface::JointStateHandle {
 public:
  HybridJointHandle() = default;

  /**
   * @brief Constructor for the HybridJointHandle.
   * @param js A standard JointStateHandle for reading the joint's state.
   * @param posDes A pointer to the variable storing the desired position command.
   * @param velDes A pointer to the variable storing the desired velocity command.
   * @param kp A pointer to the variable storing the proportional gain.
   * @param kd A pointer to the variable storing the derivative gain.
   * @param ff A pointer to the variable storing the feed-forward effort command.
   */
  HybridJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff)
      : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff) {
    if (posDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Position desired data pointer is null.");
    }
    if (velDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Velocity desired data pointer is null.");
    }
    if (kp_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
    }
    if (kd_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
    }
    if (ff_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Feedforward data pointer is null.");
    }
  }

  // --- Setters for the command values ---
  void setPositionDesired(double cmd) {
    assert(posDes_);
    *posDes_ = cmd;
  }
  void setVelocityDesired(double cmd) {
    assert(velDes_);
    *velDes_ = cmd;
  }
  void setKp(double cmd) {
    assert(kp_);
    *kp_ = cmd;
  }
  void setKd(double cmd) {
    assert(kd_);
    *kd_ = cmd;
  }
  void setFeedforward(double cmd) {
    assert(ff_);
    *ff_ = cmd;
  }

  /**
   * @brief A convenience function to set all command values at once.
   */
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff) {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }

  // --- Getters for the command values ---
  double getPositionDesired() {
    assert(posDes_);
    return *posDes_;
  }
  double getVelocityDesired() {
    assert(velDes_);
    return *velDes_;
  }
  double getKp() {
    assert(kp_);
    return *kp_;
  }
  double getKd() {
    assert(kd_);
    return *kd_;
  }
  double getFeedforward() {
    assert(ff_);
    return *ff_;
  }

 private:
  double* posDes_ = {nullptr};
  double* velDes_ = {nullptr};
  double* kp_ = {nullptr};
  double* kd_ = {nullptr};
  double* ff_ = {nullptr};
};

/**
 * @class HybridJointInterface
 * @brief The `ros_control` interface for managing a collection of hybrid joints.
 *
 * This class inherits from `HardwareResourceManager` and manages a collection of HybridJointHandle objects.
 * It is registered with the `RobotHW` and allows controllers to get access to the hybrid joint handles.
 * `ClaimResources` is used as the lock policy, meaning a controller can claim exclusive access to a joint.
 */
class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

}  // namespace legged
