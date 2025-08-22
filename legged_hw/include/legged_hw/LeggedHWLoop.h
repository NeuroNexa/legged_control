/*******************************************************************************
 * BSD 3-Clause License
 * ... (license header)
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
 * @brief 腿式机器人硬件控制循环
 *
 * 这个类封装了`ros_control`的标准控制循环逻辑。
 * 它在一个独立的线程中运行，以设定的频率调用硬件接口的 `read()` 和 `write()` 方法，
 * 并更新 `controller_manager`。
 */
class LeggedHWLoop {
  using Clock = std::chrono::high_resolution_clock; // 使用高精度时钟
  using Duration = std::chrono::duration<double>;

 public:
  /**
   * @brief 构造函数
   *
   * 创建`controller_manager`，从参数服务器加载循环频率，并启动控制循环线程。
   * @param nh ROS节点句柄
   * @param hardware_interface 指向硬件接口对象的共享指针
   */
  LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface);

  /**
   * @brief 析构函数
   *
   * 停止并等待控制循环线程结束。
   */
  ~LeggedHWLoop();

  /**
   * @brief 控制循环的更新函数
   *
   * 这个函数以固定的频率被调用。它负责：
   * 1. 计算时间差 (elapsedTime_)
   * 2. 调用 hardware_interface->read() 从硬件读取状态
   * 3. 调用 controller_manager->update() 更新所有活动的控制器
   * 4. 调用 hardware_interface->write() 将指令写入硬件
   */
  void update();

 private:
  ros::NodeHandle nh_;

  // --- 时序控制 ---
  double cycleTimeErrorThreshold_{}, loopHz_{}; // 周期时间误差阈值和循环频率
  std::thread loopThread_; // 控制循环线程
  std::atomic_bool loopRunning_{}; // 循环运行标志
  ros::Duration elapsedTime_; // 上次更新以来的经过时间
  Clock::time_point lastTime_; // 上次更新的时间点

  /**
   * @brief ROS控制器管理器
   *
   * 负责加载、卸载、启动和停止`ros_control`控制器，
   * 并在`update()`中串行执行所有正在运行的控制器。
   */
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  // 指向硬件接口的指针
  std::shared_ptr<LeggedHW> hardwareInterface_;
};

}  // namespace legged
