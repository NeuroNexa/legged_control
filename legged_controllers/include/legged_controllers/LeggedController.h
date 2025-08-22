//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @class LeggedController
 * @brief 足式机器人的主控制器。
 *
 * 此类实现了 `controller_interface::MultiInterfaceController`，并作为整个控制流程的中心枢纽。
 * 它初始化并管理所有主要组件，包括：
 * - 状态估计：从传感器数据估计机器人的当前状态。
 * - Legged Interface：为 MPC 设置最优控制问题。
 * - MPC (模型预测控制)：在未来的时间范围内计算最优的状态和输入轨迹。
 * - WBC (全身控制)：计算跟踪 MPC 轨迹所需的关节力矩。
 *
 * 控制器运行两个主循环：
 * 1. `update` 循环 (实时)：以控制器管理器的频率运行。它执行状态估计，
 *    运行 WBC，并向硬件发送命令。
 * 2. MPC 循环 (非实时)：在单独的线程中运行。它不断解决最优控制问题，
 *    为实时循环提供最新的参考轨迹。
 */
class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  /**
   * @brief 初始化控制器。
   * @param robot_hw 指向机器人硬件接口的指针。
   * @param controller_nh 控制器命名空间的 NodeHandle。
   * @return 如果初始化成功，则为 true。
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;

  /**
   * @brief 实时更新循环。
   * @param time 当前时间。
   * @param period 自上次更新以来经过的时间。
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief 控制器启动时调用。
   * @param time 控制器启动的时间。
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief 控制器停止时调用。
   */
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  /**
   * @brief 从传感器数据更新状态估计。
   */
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  // 主要组件的设置方法
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt(); // 模型参考跟踪（MRT），用于与 MPC 线程进行通信
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // 硬件和 OCS2 接口
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // 状态估计
  SystemObservation currentObservation_;
  vector_t measuredRbdState_; // 完整的刚体状态向量
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // 全身控制
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // 非线性 MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // 可视化
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

 private:
  // MPC 线程管理
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};

  // 用于基准测试的计时器
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
};

/**
 * @class LeggedCheaterController
 * @brief 一个使用地面真值状态信息的派生控制器。
 *
 * 这个“作弊”控制器用于调试和开发，通常在仿真中使用。
 * 它绕过状态估计模块，直接从模拟器（例如 Gazebo）获取完美的状态信息。
 */
class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
