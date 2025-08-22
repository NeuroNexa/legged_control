/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/30/20.
//

#pragma once

#include "legged_hw/LeggedHW.h"

#include <chrono>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

namespace legged {

/**
 * @class LeggedHWLoop
 * @brief Manages the real-time control loop for the hardware interface.
 *
 * This class follows the standard `ros_control` pattern for a hardware control loop. It is responsible for:
 * 1. Creating a `controller_manager::ControllerManager` to manage all loaded controllers.
 * 2. Spawning a dedicated real-time thread.
 * 3. In the thread, running a loop at a specified frequency that calls the standard `ros_control` sequence:
 *    - `hardware_interface->read()`: Read the latest sensor data from the hardware.
 *    - `controller_manager->update()`: Run the update function of all active controllers.
 *    - `hardware_interface->write()`: Send the latest commands to the hardware.
 */
class LeggedHWLoop {
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

 public:
  /**
   * @brief Constructor for the LeggedHWLoop.
   *
   * Initializes the controller manager, gets the loop frequency from the parameter server,
   * and starts the real-time control loop in a new thread.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A shared pointer to the hardware interface.
   */
  LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface);

  /**
   * @brief Destructor for the LeggedHWLoop.
   *
   * Stops the control loop thread and waits for it to join.
   */
  ~LeggedHWLoop();

  /**
   * @brief The core update function of the control loop.
   *
   * This method performs the standard `ros_control` read-update-write cycle.
   */
  void update();

 private:
  ros::NodeHandle nh_;

  // Timing
  double cycleTimeErrorThreshold_{}, loopHz_{};
  std::thread loopThread_;
  std::atomic_bool loopRunning_{};
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;

  // The ROS Controller Manager
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  // The hardware interface
  std::shared_ptr<LeggedHW> hardwareInterface_;
};

}  // namespace legged
