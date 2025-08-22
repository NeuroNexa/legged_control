//
// Created by qiayuan on 23-1-30.
//

#pragma once
#include <ros/ros.h>

#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <utility>

namespace legged {

using namespace ocs2;

/**
 * @brief 腿式机器人自碰撞可视化类
 *
 * 该类继承自 OCS2 的 `GeometryInterfaceVisualization`，专门用于可视化机器人各连杆之间的距离，
 * 以便直观地检查自碰撞约束是否生效。
 */
class LeggedSelfCollisionVisualization : public GeometryInterfaceVisualization {
 public:
  /**
   * @brief 构造函数
   * @param pinocchioInterface Pinocchio库的接口，用于运动学计算
   * @param geometryInterface Pinocchio库的几何模型接口，用于碰撞检测
   * @param mapping 质心模型状态到Pinocchio关节状态的映射关系
   * @param nh ROS节点句柄
   * @param maxUpdateFrequency 最大更新频率，用于控制发布可视化的频率
   */
  LeggedSelfCollisionVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping, ros::NodeHandle& nh, scalar_t maxUpdateFrequency = 50.0)
      : mappingPtr_(mapping.clone()),
        GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), nh, "odom"),
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

  /**
   * @brief 更新并发布可视化信息
   * @param observation 当前的系统观测值
   */
  void update(const SystemObservation& observation) {
    // 检查是否达到了发布的最小时间间隔，以避免发布频率过高
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
      lastTime_ = observation.time;

      // 从质心模型状态 observation.state 映射到 Pinocchio 的关节位置
      const auto pinocchioJointPosition = mappingPtr_->getPinocchioJointPosition(observation.state);
      // 调用基类的方法，发布连杆间的距离可视化标记 (Marker)
      publishDistances(pinocchioJointPosition);
    }
  }

 private:
  // 状态映射关系的指针
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;

  // 用于控制发布频率的时间戳
  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged
