//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
/**
 * @brief 接触传感器的句柄（Handle）
 *
 * 用于获取单个接触传感器的状态
 */
class ContactSensorHandle {
 public:
  ContactSensorHandle() = default;

  /**
   * @brief 构造函数
   * @param name 传感器名称
   * @param isContact 指向接触状态的指针 (true表示接触, false表示未接触)
   */
  ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact) {
    if (isContact == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
    }
  }

  /**
   * @brief 获取传感器名称
   * @return 传感器的名称
   */
  std::string getName() const { return name_; }

  /**
   * @brief 获取接触状态
   * @return 如果正在接触则为 true
   */
  bool isContact() const {
    assert(isContact_); // 断言指针不为空
    return *isContact_;
  }

 private:
  std::string name_; // 传感器名称

  const bool* isContact_ = {nullptr}; // 指向接触状态的常量指针
};

/**
 * @brief 接触传感器接口
 *
 * 用于管理多个接触传感器句柄的ROS Control接口
 */
class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

}  // namespace legged
