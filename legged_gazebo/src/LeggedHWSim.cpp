/*******************************************************************************
 * BSD 3-Clause License
 * ... (license header)
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#include "legged_gazebo/LeggedHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace legged {
/**
 * @brief 初始化仿真硬件接口
 */
bool LeggedHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                          const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) {
  // 调用基类的初始化函数
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

  // 注册自定义的硬件接口
  registerInterface(&hybridJointInterface_);

  // 从默认的EffortJointInterface中获取关节句柄，并用它们来创建和注册我们自定义的HybridJointHandle
  std::vector<std::string> names = ej_interface_.getNames();
  for (const auto& name : names) {
    hybridJointDatas_.push_back(HybridJointData{.joint_ = ej_interface_.getHandle(name)});
    HybridJointData& back = hybridJointDatas_.back();
    // 为每个关节注册一个混合模式的句柄，它包含了对期望位置、速度、Kp, Kd和前馈力矩的引用
    hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
    // 为每个关节创建一个指令缓冲区，用于模拟延迟
    cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
  }

  // 注册IMU接口并解析参数
  registerInterface(&imuSensorInterface_);
  XmlRpc::XmlRpcValue xmlRpcValue;
  if (!model_nh.getParam("gazebo/imus", xmlRpcValue)) {
    ROS_WARN("No imu specified");
  } else {
    parseImu(xmlRpcValue, parent_model);
  }

  // 获取延迟和接触传感器参数
  if (!model_nh.getParam("gazebo/delay", delay_)) {
    delay_ = 0.;
  }
  if (!model_nh.getParam("gazebo/contacts", xmlRpcValue)) {
    ROS_WARN("No contacts specified");
  } else {
    parseContacts(xmlRpcValue);
  }

  // 获取Gazebo的接触管理器
  contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();
  contactManager_->SetNeverDropContacts(true);
  return ret;
}

/**
 * @brief 从仿真环境中读取数据
 */
void LeggedHWSim::readSim(ros::Time time, ros::Duration period) {
  // --- 读取关节状态 ---
  // DefaultRobotHWSim::readSim() 会给速度带来一个偏移，所以这里重写了这部分逻辑
  for (unsigned int j = 0; j < n_dof_; j++) {
    double position = sim_joints_[j]->Position(0);
    // 通过差分计算速度
    joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();
    if (time == ros::Time(period.toSec())) { // 仿真重置时，速度为0
      joint_velocity_[j] = 0;
    }
    // 对旋转关节进行角度最短路径处理
    if (joint_types_[j] == urdf::Joint::PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
    }
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  // --- 读取IMU数据 ---
  for (auto& imu : imuDatas_) {
    ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X(); // ...
    imu.ori_[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X(); // ...
    // 计算线加速度，并移除重力影响
    ignition::math::Vector3d gravity = {0., 0., -9.81};
    ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X(); // ...
  }

  // --- 读取接触传感器数据 ---
  for (auto& state : name2contact_) {
    state.second = false; // 每一步都先重置接触状态
  }
  // 遍历Gazebo中的所有接触点
  for (const auto& contact : contactManager_->GetContacts()) {
    // 确保接触点是当前仿真步的
    if (static_cast<uint32_t>(contact->time.sec) != (time - period).sec ||
        static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec) {
      continue;
    }
    // 检查接触的两个碰撞体中是否有我们关心的足底连杆
    std::string linkName = contact->collision1->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
    linkName = contact->collision2->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
  }

  // --- 安全措施 ---
  // 当没有控制器加载时，将所有指令设为0，避免机器人乱动
  for (auto& cmd : joint_effort_command_) cmd = 0;
  for (auto& cmd : joint_velocity_command_) cmd = 0;
  for (auto& joint : hybridJointDatas_) {
    joint.posDes_ = joint.joint_.getPosition();
    joint.velDes_ = joint.joint_.getVelocity();
    joint.kp_ = 0.;
    joint.kd_ = 0.;
    joint.ff_ = 0.;
  }
}

/**
 * @brief 向仿真环境写入数据
 */
void LeggedHWSim::writeSim(ros::Time time, ros::Duration period) {
  for (auto joint : hybridJointDatas_) {
    // --- 模拟通信延迟 ---
    auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
    if (time == ros::Time(period.toSec())) {  // 仿真重置时清空缓冲区
      buffer.clear();
    }
    // 移除过时的指令
    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }
    // 将当前指令存入缓冲区前端
    buffer.push_front(HybridJointCommand{
        .stamp_ = time, .posDes_ = joint.posDes_, .velDes_ = joint.velDes_, .kp_ = joint.kp_, .kd_ = joint.kd_, .ff_ = joint.ff_});

    // --- 应用指令到Gazebo关节 ---
    // 获取缓冲区末尾（最旧的，也就是延迟后的）指令
    const auto& cmd = buffer.back();
    // 计算PD控制力矩 + 前馈力矩，并应用到仿真关节上
    joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
                            cmd.ff_);
  }
  // 调用基类的writeSim来处理其他类型的关节（如果有的话）
  DefaultRobotHWSim::writeSim(time, period);
}

/**
 * @brief 解析IMU配置
 */
void LeggedHWSim::parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel) {
  // ... (从XmlRpcValue中解析参数)
  std::string frameId = imuDatas[it->first]["frame_id"];
  gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
  // ... (创建ImuData结构体)
  imuDatas_.push_back((ImuData{/*...*/}));
  // 注册IMU传感器接口
  ImuData& imuData = imuDatas_.back();
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_, imuData.angularVel_, imuData.angularVelCov_, imuData.linearAcc_, imuData.linearAccCov_));
}

/**
 * @brief 解析接触传感器配置
 */
void LeggedHWSim::parseContacts(XmlRpc::XmlRpcValue& contactNames) {
  // ... (从XmlRpcValue中解析连杆名称)
  for (int i = 0; i < contactNames.size(); ++i) {
    std::string name = contactNames[i];
    name2contact_.insert(std::make_pair(name, false));
    // 注册接触传感器接口
    contactSensorInterface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
  }
  registerInterface(&contactSensorInterface_);
}

}  // namespace legged

// 导出该类作为gazebo_ros_control的插件
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros_control::RobotHWSim)
// 注册默认的Gazebo ROS Control插件
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)
