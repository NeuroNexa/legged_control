//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"

// 根据定义的SDK版本，包含对应的手柄数据结构头文件
#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/joystick.h"
#endif

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

namespace legged {
/**
 * @brief 初始化硬件接口
 */
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // 调用基类的init方法
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // 从参数服务器获取功率限制
  robot_hw_nh.getParam("power_limit", powerLimit_);

  // 注册关节、IMU和接触传感器的ros_control接口
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  // --- 初始化Unitree SDK ---
  // 根据SDK版本初始化UDP通信
#ifdef UNITREE_SDK_3_3_1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
#elif UNITREE_SDK_3_8_0
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
#endif
  udp_->InitCmdData(lowCmd_);

  // 根据机器人类型初始化安全保护模块
  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
#ifdef UNITREE_SDK_3_3_1
  if (robot_type == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type == "aliengo") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  }
#elif UNITREE_SDK_3_8_0
  if (robot_type == "go1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Go1);
  }
#endif
  else {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }

  // 初始化ROS话题发布器
  joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
  contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(std::string("/contact"), 10);
  return true;
}

/**
 * @brief 从硬件读取数据
 */
void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  // 1. 通过UDP接收底层状态数据
  udp_->Recv();
  udp_->GetRecv(lowState_);

  // 2. 将接收到的SDK数据填充到ros_control的硬件接口数据结构中
  // 更新关节数据
  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = lowState_.motorState[i].q;
    jointData_[i].vel_ = lowState_.motorState[i].dq;
    jointData_[i].tau_ = lowState_.motorState[i].tauEst;
  }

  // 更新IMU数据 (注意SDK和ROS的四元数顺序不同)
  imuData_.ori_[0] = lowState_.imu.quaternion[1]; // x
  imuData_.ori_[1] = lowState_.imu.quaternion[2]; // y
  imuData_.ori_[2] = lowState_.imu.quaternion[3]; // z
  imuData_.ori_[3] = lowState_.imu.quaternion[0]; // w
  imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
  // ...
  imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
  // ...

  // 更新接触传感器数据
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] = lowState_.footForce[i] > contactThreshold_;
  }

  // 3. 安全措施：当没有控制器设置指令时，将前馈和期望速度设为0，避免机器人意外移动
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  // 4. 更新并发布手柄和接触力数据
  updateJoystick(time);
  updateContact(time);
}

/**
 * @brief 向硬件写入数据
 */
void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // 1. 从ros_control的硬件接口中获取指令，填充到SDK的指令结构体中
  for (int i = 0; i < 12; ++i) {
    lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
    lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
    lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
    lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
    lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
  }

  // 2. 调用SDK的安全保护函数
  safety_->PositionLimit(lowCmd_); // 位置限制
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_); // 功率限制

  // 3. 通过UDP发送指令
  udp_->SetSend(lowCmd_);
  udp_->Send();
}

/**
 * @brief 注册关节接口
 */
bool UnitreeHW::setupJoints() {
  // 遍历URDF中的所有关节
  for (const auto& joint : urdfModel_->joints_) {
    // 根据关节名称确定其在SDK数据数组中的索引
    int leg_index = 0, joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) leg_index = UNITREE_LEGGED_SDK::FR_;
    // ...
    if (joint.first.find("HAA") != std::string::npos) joint_index = 0;
    // ...

    int index = leg_index * 3 + joint_index;
    // 创建并注册JointStateHandle和HybridJointHandle
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_, &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_, &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

/**
 * @brief 注册IMU接口
 */
bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_, imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_, imuData_.linearAccCov_));
  // 设置默认的协方差值
  imuData_.oriCov_[0] = 0.0012;
  // ...
  return true;
}

/**
 * @brief 注册接触传感器接口
 */
bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

/**
 * @brief 更新并发布手柄数据
 */
void UnitreeHW::updateJoystick(const ros::Time& time) {
  // 限速发布
  if ((time - lastJoyPub_).toSec() < 1 / 50.) return;
  lastJoyPub_ = time;
  // 从lowState_中解析手柄数据并填充到sensor_msgs::Joy消息中
  xRockerBtnDataStruct keyData;
  memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
  sensor_msgs::Joy joyMsg;
  // ...
  joyPublisher_.publish(joyMsg);
}

/**
 * @brief 更新并发布原始接触力数据
 */
void UnitreeHW::updateContact(const ros::Time& time) {
  if ((time - lastContactPub_).toSec() < 1 / 50.) return;
  lastContactPub_ = time;
  std_msgs::Int16MultiArray contactMsg;
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactMsg.data.push_back(lowState_.footForce[i]);
  }
  contactPublisher_.publish(contactMsg);
}

}  // namespace legged
