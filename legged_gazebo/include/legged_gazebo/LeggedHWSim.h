/*******************************************************************************
 * BSD 3-Clause License
 * ... (license header)
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {

/**
 * @brief 混合关节的数据结构，用于仿真
 */
struct HybridJointData {
  hardware_interface::JointHandle joint_; // ros_control的标准关节句柄
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{}; // 期望位置、速度、增益和前馈力矩
};

/**
 * @brief 带有时间戳的混合关节指令
 */
struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

/**
 * @brief IMU传感器的数据结构，用于仿真
 */
struct ImuData {
  gazebo::physics::LinkPtr linkPtr_; // 指向Gazebo中IMU连杆的指针
  double ori_[4];
  double oriCov_[9];
  double angularVel_[3];
  double angularVelCov_[9];
  double linearAcc_[3];
  double linearAccCov_[9];
};

/**
 * @brief 腿式机器人的Gazebo硬件仿真接口
 *
 * 该类继承自`gazebo_ros_control`的`DefaultRobotHWSim`，
 * 作为一个插件，实现了`ros_control`与Gazebo物理引擎之间的通信。
 * 它负责：
 * 1. 从Gazebo读取关节状态、IMU数据、接触力数据，并填充到ros_control的硬件接口中。
 * 2. 从ros_control的硬件接口获取关节指令，并将其应用到Gazebo的关节上。
 */
class LeggedHWSim : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  /**
   * @brief 初始化仿真接口
   *
   * 在Gazebo加载模型时被调用。负责解析URDF，设置和注册硬件接口。
   * @return 如果初始化成功则为 true
   */
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  /**
   * @brief 从仿真环境中读取数据
   *
   * 在每个仿真周期的开始被调用。负责从Gazebo获取传感器数据。
   */
  void readSim(ros::Time time, ros::Duration period) override;

  /**
   * @brief 向仿真环境写入数据
   *
   * 在每个仿真周期的末尾被调用。负责将控制器指令应用到Gazebo模型。
   */
  void writeSim(ros::Time time, ros::Duration period) override;

 private:
  /**
   * @brief 解析配置文件中的IMU信息
   */
  void parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel);

  /**
   * @brief 解析配置文件中的接触传感器信息
   */
  void parseContacts(XmlRpc::XmlRpcValue& contactNames);

  // --- ros_control硬件接口 ---
  HybridJointInterface hybridJointInterface_;
  ContactSensorInterface contactSensorInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;

  // --- Gazebo相关 ---
  gazebo::physics::ContactManager* contactManager_{}; // Gazebo的接触管理器，用于检测足底接触

  // --- 数据缓冲区 ---
  std::list<HybridJointData> hybridJointDatas_; // 混合关节数据列表
  std::list<ImuData> imuDatas_; // IMU数据列表
  // 指令缓冲区，用于模拟通信延迟
  std::unordered_map<std::string, std::deque<HybridJointCommand> > cmdBuffer_;
  // 接触状态的映射
  std::unordered_map<std::string, bool> name2contact_;

  // 通信延迟时间 (s)
  double delay_{};
};

}  // namespace legged
