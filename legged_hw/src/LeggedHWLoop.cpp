
//
// Created by qiayuan on 1/24/22.
//
#include "legged_hw/LeggedHWLoop.h"

namespace legged {
LeggedHWLoop::LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // Create the controller manager
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // Load ROS parameters
  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("~");
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  if (error > 0) {
    std::string error_message =
        "Could not retrieve one of the required parameters: loop_frequency, cycle_time_error_threshold, or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get the current time for use with the first update
  lastTime_ = Clock::now();

  // Start the control loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });

  // Set the thread priority for real-time performance
  sched_param sched{.sched_priority = threadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN(
        "Failed to set thread priority. This is not an error, but may lead to lower performance. Possible reason: user does not have "
        "permissions to set real-time priorities.");
  }
}

void LeggedHWLoop::update() {
  const auto currentTime = Clock::now();
  const Duration desiredDuration(1.0 / loopHz_);

  // Calculate the time elapsed since the last update
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check for excessive delay in the control loop
  const double cycleTimeError = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycleTimeError > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycleTimeError - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // --- The ros_control loop ---
  // 1. Read state from the hardware
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // 2. Update the controllers
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // 3. Write commands to the hardware
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);
  // -------------------------

  // Sleep to maintain the desired loop frequency
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

LeggedHWLoop::~LeggedHWLoop() {
  // Stop the control loop thread
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

}  // namespace legged
