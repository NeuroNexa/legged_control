//
// Created by qiayuan on 2022/7/16.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_interface/LeggedInterface.h"
#include "legged_interface/LeggedRobotPreComputation.h"
#include "legged_interface/constraint/FrictionConeConstraint.h"
#include "legged_interface/constraint/LeggedSelfCollisionConstraint.h"
#include "legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "legged_interface/constraint/ZeroForceConstraint.h"
#include "legged_interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "legged_interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "legged_interface/initialization/LeggedRobotInitializer.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace legged {
/**
 * @brief LeggedInterface 构造函数
 */
LeggedInterface::LeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                 bool useHardFrictionConeConstraint)
    : useHardFrictionConeConstraint_(useHardFrictionConeConstraint) {
  // 检查配置文件是否存在
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedInterface] Task file not found: " + taskFilePath.string());
  }
  // ... (检查 urdfFile 和 referenceFile)

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // 从配置文件加载所有设置
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
  ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 设置最优控制问题
 */
void LeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                 bool verbose) {
  // 1. 设置模型 (Pinocchio, Centroidal)
  setupModel(taskFile, urdfFile, referenceFile, verbose);

  // 2. 设置初始状态
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  // 3. 设置参考管理器
  setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);

  // 4. 创建最优控制问题实例
  problemPtr_ = std::make_unique<OptimalControlProblem>();

  // 5. 设置动力学
  problemPtr_->dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_);

  // 6. 设置代价函数
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

  // 7. 设置约束
  // a. 摩擦锥约束
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];
    auto eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    // 根据配置选择硬约束或软约束
    if (useHardFrictionConeConstraint_) {
      problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", getFrictionConeConstraint(i, frictionCoefficient));
    } else {
      problemPtr_->softConstraintPtr->add(footName + "_frictionCone", getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
    }
    // b. 摆动腿零力约束
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, i, centroidalModelInfo_)));
    // c. 支撑腿零速约束
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i));
    // d. 摆动腿法向速度约束
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, *eeKinematicsPtr, i)));
  }

  // e. 自碰撞约束
  problemPtr_->stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));

  // 8. 设置预计算模块
  setupPreComputation(taskFile, urdfFile, referenceFile, verbose);

  // 9. 设置前向推演 (Rollout)
  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  // 10. 设置初始化器
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_ = std::make_unique<LeggedRobotInitializer>(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 设置机器人模型
 */
void LeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                 bool /*verbose*/) {
  // Pinocchio 接口
  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

  // 质心动力学模型信息
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 设置参考管理器
 */
void LeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 设置预计算
 */
void LeggedInterface::setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose) {
  problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);
}

/******************************************************************************************************/
// ... 其他辅助函数的实现 ...
/******************************************************************************************************/

}  // namespace legged
