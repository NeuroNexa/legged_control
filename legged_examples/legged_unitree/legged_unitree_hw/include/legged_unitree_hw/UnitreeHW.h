//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

// --- Unitree SDK ---
// 根据CMake中定义的宏，包含不同版本的Unitree SDK头文件。
// 这种方式使得同一套代码可以适配不同版本的硬件SDK。
#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/safety.h"
#include "unitree_legged_sdk_3_3_1/udp.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/safety.h"
#include "unitree_legged_sdk_3_8_0/udp.h"
#endif

namespace legged {
// 定义接触传感器的名称和顺序
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

/**
 * @brief 用于存储单个电机状态和指令的数据结构
 */
struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // 状态: 位置, 速度, 力矩
  double posDes_, velDes_, kp_, kd_, ff_;  // 指令: 期望位置, 期望速度, Kp, Kd, 前馈力矩
};

/**
 * @brief 用于存储IMU数据的数据结构
 */
struct UnitreeImuData {
  double ori_[4];            // 四元数 [w, x, y, z]
  double oriCov_[9];         // 姿态协方差
  double angularVel_[3];     // 角速度
  double angularVelCov_[9];  // 角速度协方差
  double linearAcc_[3];      // 线加速度
  double linearAccCov_[9];   // 线加速度协方差
};

/**
 * @brief Unitree 硬件接口类
 *
 * 继承自通用的 `LeggedHW`，实现了与Unitree机器人底层SDK的通信。
 * 它负责从SDK读取传感器数据（关节、IMU、足底力），并向SDK发送控制指令。
 */
class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  /**
   * @brief 初始化硬件接口
   *
   * 从参数服务器获取配置，设置并注册ros_control的各种硬件接口（关节、IMU、接触传感器）。
   * @param root_nh 全局ROS节点句柄
   * @param robot_hw_nh 控制器私有的ROS节点句柄
   * @return 如果初始化成功则为 true
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * @brief 从硬件读取数据
   *
   * 调用Unitree SDK的UDP接收函数，获取机器人状态，并更新到 `jointData_`, `imuData_` 等成员变量中。
   * @param time 当前时间
   * @param period 控制周期
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief 向硬件写入数据
   *
   * 从ros_control的关节句柄中获取指令，填充到底层SDK的指令结构体中，
   * 并调用Unitree SDK的UDP发送函数将指令发送给机器人。
   * @param time 当前时间
   * @param period 控制周期
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief 更新并发布手柄数据
   *
   * 从底层状态中读取遥控器数据并发布为ROS消息。
   */
  void updateJoystick(const ros::Time& time);

  /**
   * @brief 更新并发布接触状态
   *
   * 将SDK中的足底力数据根据阈值转换为布尔型的接触状态，并发布。
   */
  void updateContact(const ros::Time& time);

 private:
  /**
   * @brief 注册关节接口
   */
  bool setupJoints();

  /**
   * @brief 注册IMU传感器接口
   */
  bool setupImu();

  /**
   * @brief 注册接触传感器接口
   */
  bool setupContactSensor(ros::NodeHandle& nh);

  // --- Unitree SDK相关 ---
  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_; // UDP通信对象
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_; // 安全保护对象
  UNITREE_LEGGED_SDK::LowState lowState_{}; // 接收底层状态的结构体
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};   // 发送底层指令的结构体

  // --- 硬件数据缓冲区 ---
  UnitreeMotorData jointData_[12]{};
  UnitreeImuData imuData_{};
  bool contactState_[4]{};

  // --- 配置参数 ---
  int powerLimit_{};
  int contactThreshold_{};

  // --- ROS Publisher ---
  ros::Publisher joyPublisher_;
  ros::Publisher contactPublisher_;
  ros::Time lastJoyPub_, lastContactPub_;
};

}  // namespace legged
