/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include "legged_interface/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief LeggedRobotPreComputation 构造函数
 */
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      settings_(std::move(settings)) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief LeggedRobotPreComputation 拷贝构造函数
 */
LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      info_(rhs.info_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      settings_(rhs.settings_) {
  eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 执行预计算
 *
 * @param request 请求集合，指明需要计算哪些量
 * @param t 当前时间
 * @param x 当前状态
 * @param u 当前输入
 */
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  // 如果请求中不包含代价或约束，则无需进行预计算
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // 定义一个lambda函数，用于为法向速度约束生成配置
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    // 约束形式: 1.0 * v_z - v_z_ref = 0
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    // 如果使用了位置误差增益，则额外增加一个位置项
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  // 如果请求中包含约束，则为所有脚生成并缓存约束配置
  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
    }
  }

  // --- 更新Pinocchio模型 ---
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  // 从OCS2的状态向量x中，映射出Pinocchio的广义坐标q
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);

  // 如果需要计算线性逼近（即导数），则需要计算雅可比等
  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    // 更新质心动力学模型及其导数
    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  } else { // 如果只需要计算值，则只需更新运动学
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
