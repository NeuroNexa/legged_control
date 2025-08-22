//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {

/**
 * @class ContactSensorHandle
 * @brief A handle to a single contact sensor.
 *
 * This class follows the standard `ros_control` pattern for a hardware handle. It provides a named
 * interface to a single resource, in this case, a boolean contact state. The hardware abstraction
 * layer (e.g., LeggedHW or LeggedHWSim) will create these handles and register them with the
 * ContactSensorInterface. Controllers can then get a handle to a specific sensor by name to read its state.
 */
class ContactSensorHandle {
 public:
  ContactSensorHandle() = default;

  /**
   * @brief Constructor for the ContactSensorHandle.
   * @param name The name of the contact sensor (e.g., "LF_FOOT_CONTACT").
   * @param isContact A pointer to the boolean variable that stores the contact state.
   */
  ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact) {
    if (isContact == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
    }
  }

  /** @brief Gets the name of the sensor. */
  std::string getName() const { return name_; }

  /** @brief Gets the current contact state. */
  bool isContact() const {
    assert(isContact_);
    return *isContact_;
  }

 private:
  std::string name_;
  const bool* isContact_ = {nullptr};
};

/**
 * @class ContactSensorInterface
 * @brief The `ros_control` interface for managing a collection of contact sensors.
 *
 * This class inherits from `HardwareResourceManager` and manages a collection of ContactSensorHandle objects.
 * It is registered with the `RobotHW` and allows controllers to get access to the contact sensor handles.
 */
class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

}  // namespace legged
