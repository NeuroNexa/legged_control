//
// Created by qiayuan on 1/24/22.
//
#include "legged_hw/LeggedHWLoop.h"

namespace legged {
/**
 * @brief LeggedHWLoop 构造函数
 *
 * @param nh ROS节点句柄
 * @param hardware_interface 指向硬件接口对象的共享指针
 */
LeggedHWLoop::LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // 创建控制器管理器
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // 从参数服务器加载循环频率、误差阈值和线程优先级
  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("~"); // 私有节点句柄
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  if (error > 0) {
    std::string error_message = "Could not retrieve one of the required parameters: loop_hz, cycle_time_error_threshold, or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // 获取当前时间，用于第一次更新
  lastTime_ = Clock::now();

  // 创建并启动控制循环线程
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  // 设置线程为实时调度策略（SCHED_FIFO）并设置优先级
  sched_param sched{.sched_priority = threadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN("Failed to set threads priority (one possible reason could be that the user and the group permissions are not set properly.).\n");
  }
}

/**
 * @brief 控制循环的更新函数
 */
void LeggedHWLoop::update() {
  const auto currentTime = Clock::now();
  // 计算期望的周期时长
  const Duration desiredDuration(1.0 / loopHz_);

  // 计算实际经过的时间
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // 检查实际周期是否超出误差阈值
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // --- ros_control的 "read-update-write" 循环 ---

  // 1. 读取（Input）
  //    获取硬件的当前状态（关节位置、速度等）
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // 2. 更新（Control）
  //    让控制器管理器根据当前状态计算新的指令
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // 3. 写入（Output）
  //    将新的指令发送给硬件
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // --- 睡眠以保持固定频率 ---
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

/**
 * @brief LeggedHWLoop 析构函数
 */
LeggedHWLoop::~LeggedHWLoop() {
  loopRunning_ = false; // 设置循环标志为false，使线程退出while循环
  if (loopThread_.joinable()) {
    loopThread_.join(); // 等待线程结束
  }
}

}  // namespace legged
