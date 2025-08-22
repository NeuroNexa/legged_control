#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-non-private-member-variables-in-classes"
//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief 腿式机器人OCS2接口类
 *
 * 该类是整个OCS2最优控制框架的核心。它负责：
 * 1. 从配置文件加载所有设置（模型、MPC、求解器等）。
 * 2. 构建`OptimalControlProblem`对象，该对象封装了最优控制问题的所有要素：
 *    - 动力学模型 (Dynamics)
 *    - 代价函数 (Cost)
 *    - 约束 (Constraint)
 * 3. 创建和管理参考轨迹（ReferenceManager）、初始化器（Initializer）等。
 * 4. 提供对Pinocchio模型、质心模型等机器人模型的访问。
 */
class LeggedInterface : public RobotInterface {
 public:
  /**
   * @brief 构造函数
   * @param taskFile 任务配置文件路径
   * @param urdfFile URDF文件路径
   * @param referenceFile 参考配置文件路径
   * @param useHardFrictionConeConstraint 是否使用摩擦锥硬约束
   */
  LeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = false);

  ~LeggedInterface() override = default;

  /**
   * @brief 设置最优控制问题
   *
   * 这是主要的设置函数，会调用其他受保护的setup函数来分步构建问题。
   */
  virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose);

  // --- Getters ---
  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }
  const ipm::Settings& ipmSettings() { return ipmSettings_; }
  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  PinocchioGeometryInterface& getGeometryInterface() { return *geometryInterfacePtr_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }
  const Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

 protected:
  /**
   * @brief 设置机器人模型（Pinocchio, Centroidal）
   */
  virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  /**
   * @brief 设置参考管理器
   */
  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);
  /**
   * @brief 设置预计算模块
   */
  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                   bool verbose);

  /**
   * @brief 从文件加载步态序列
   */
  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  // --- 用于创建代价和约束的辅助函数 ---
  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);
  static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose);
  std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
  std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient, const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& footNames, const std::string& modelName);
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex);
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile, const std::string& prefix, bool verbose);

  // --- 成员变量 ---
  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  ddp::Settings ddpSettings_;
  sqp::Settings sqpSettings_;
  ipm::Settings ipmSettings_;
  const bool useHardFrictionConeConstraint_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged

#pragma clang diagnostic pop
