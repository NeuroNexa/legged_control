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
 * @brief 腿式机器人主控制器
 *
 * 这是一个 `ros_control` 的控制器，它实现了NMPC-WBC（非线性模型预测控制-全身控制）的控制框架。
 * 它从硬件接口（关节、IMU、足底接触传感器）读取数据，通过状态估计模块估计机器人状态，
 * 然后运行MPC来生成期望的运动轨迹，最后由WBC计算出最终的关节指令。
 *
 * @tparam HybridJointInterface 混合关节接口（位置+力矩前馈）
 * @tparam ImuSensorInterface IMU传感器接口
 * @tparam ContactSensorInterface 足底接触传感器接口
 */
class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;

  /**
   * @brief 控制器初始化
   * @param robot_hw 机器人硬件抽象指针
   * @param controller_nh 控制器私有的NodeHandle
   * @return 如果初始化成功则为 true
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;

  /**
   * @brief 控制器更新，在每个控制周期被调用
   * @param time 当前时间
   * @param period 控制周期
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief 控制器启动
   * @param time 启动时间
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief 控制器停止
   * @param time 停止时间
   */
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  /**
   * @brief 更新状态估计
   */
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 设置 OCS2 LeggedInterface
   *
   * LeggedInterface 是 OCS2 框架的核心，它定义了最优控制问题（动力学、代价、约束）。
   */
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  /**
   * @brief 设置 MPC (模型预测控制器)
   */
  virtual void setupMpc();

  /**
   * @brief 设置 MRT (模型参考跟踪)
   *
   * MRT 负责将MPC的计算结果（在单独的线程中）与主控制循环同步。
   */
  virtual void setupMrt();

  /**
   * @brief 设置状态估计器
   */
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // --- 主要模块指针 ---

  // OCS2 接口
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_; // 末端执行器运动学

  // 硬件接口句柄
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // 状态估计
  SystemObservation currentObservation_; // 当前的系统观测值
  vector_t measuredRbdState_; // 测量到的RBD（刚体动力学）状态
  std::shared_ptr<StateEstimateBase> stateEstimate_; // 状态估计器基类指针
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_; // RBD状态与质心模型状态转换工具

  // 全身控制器 (WBC)
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_; // 安全检查器

  // 非线性 MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // 可视化
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_; // 用于发布观测数据

 private:
  // --- 多线程与同步 ---
  std::thread mpcThread_; // MPC计算线程
  std::atomic_bool controllerRunning_{}, mpcRunning_{}; // 控制器和MPC运行状态标志
  benchmark::RepeatedTimer mpcTimer_; // MPC计时器，用于性能分析
  benchmark::RepeatedTimer wbcTimer_; // WBC计时器
};


/**
 * @brief “作弊”控制器
 *
 * 继承自 LeggedController，但重载了 `setupStateEstimate` 方法。
 * 这个控制器不使用真实的状态估计器，而是直接从仿真环境中获取完美的机器人状态（“作弊”）。
 * 主要用于算法调试，以排除状态估计误差的干扰。
 */
class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
