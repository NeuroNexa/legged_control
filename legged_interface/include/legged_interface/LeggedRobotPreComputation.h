/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <ocs2_legged_robot/common/ModelSettings.h>

#include "legged_interface/constraint/EndEffectorLinearConstraint.h"
#include "legged_interface/constraint/SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 腿式机器人的预计算类
 *
 * 在OCS2中，`PreComputation`模块用于在计算动力学、代价和约束之前，预先计算一些共享的值，
 * 以避免在不同的地方重复计算，提高效率。
 *
 * 这个类专门为腿式机器人服务，它会：
 * 1. 更新Pinocchio模型的数据（如正运动学）。
 * 2. 根据当前的步态和时间，从`SwingTrajectoryPlanner`获取摆动腿的轨迹信息，
 *    并将其缓存起来，供`NormalVelocityConstraint`等约束使用。
 */
class LeggedRobotPreComputation : public PreComputation {
 public:
  /**
   * @brief 构造函数
   * @param pinocchioInterface Pinocchio模型接口
   * @param info 质心模型信息
   * @param swingTrajectoryPlanner 摆动腿轨迹规划器
   * @param settings 模型设置
   */
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);

  ~LeggedRobotPreComputation() override = default;

  /**
   * @brief 克隆函数，用于创建该对象的深拷贝
   */
  LeggedRobotPreComputation* clone() const override { return new LeggedRobotPreComputation(*this); }

  /**
   * @brief 请求计算
   *
   * 在每个时间点、状态和输入下被调用，以执行预计算。
   * @param request 请求集合，指明需要计算哪些量（例如，动力学、约束、代价）
   * @param t 当前时间
   * @param x 当前状态
   * @param u 当前输入
   */
  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * @brief 获取末端执行器法向速度约束的配置
   * @return 约束配置的向量
   */
  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  /**
   * @brief 拷贝构造函数
   */
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other);

 private:
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  const ModelSettings settings_;

  // 缓存的末端执行器法向速度约束配置
  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

}  // namespace legged_robot
}  // namespace ocs2
