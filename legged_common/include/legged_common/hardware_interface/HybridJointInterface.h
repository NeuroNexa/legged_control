//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
/**
 * @brief 混合关节的句柄 (Handle)
 *
 * 继承自 hardware_interface::JointStateHandle,
 * 除了提供关节状态（位置、速度、力）的访问外，
 * 还提供了对期望位置、期望速度、Kp、Kd和前馈力矩的设置和获取。
 * 这种混合控制模式常见于腿式机器人，结合了位置控制和力矩前馈。
 */
class HybridJointHandle : public hardware_interface::JointStateHandle {
 public:
  HybridJointHandle() = default;

  /**
   * @brief 构造函数
   * @param js JointStateHandle, 提供关节状态的访问
   * @param posDes 指向期望位置的指针
   * @param velDes 指向期望速度的指针
   * @param kp 指向Kp增益的指针
   * @param kd 指向Kd增益的指针
   * @param ff 指向力矩前馈的指针
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
  // --- 设置指令 ---
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
   * @brief 一次性设置所有控制指令
   */
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff) {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }

  // --- 获取指令 ---
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
  // 指向控制指令数据的指针
  double* posDes_ = {nullptr};
  double* velDes_ = {nullptr};
  double* kp_ = {nullptr};
  double* kd_ = {nullptr};
  double* ff_ = {nullptr};
};

/**
 * @brief 混合关节接口
 *
 * 用于管理多个混合关节句柄的ROS Control接口
 * ClaimResources 表示这个接口会“声明”并拥有它所管理的资源（关节），
 * 其他接口不能再声明同样的资源。
 */
class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

}  // namespace legged
