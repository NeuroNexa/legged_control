#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-non-private-member-variables-in-classes"
//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.hh>
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
 * @class LeggedInterface
 * @brief LeggedInterface 类
 * 它继承自 ocs2::RobotInterface 类，负责为足式机器人设置整个最优控制问题。
 * 这包括定义机器人模型、约束、代价函数和求解器设置。
 * 它充当了高级控制器和 ocs2 库之间的桥梁。
 */
class LeggedInterface : public RobotInterface {
 public:
  /**
   * @brief LeggedInterface 类的构造函数。
   * @param taskFile 任务配置文件的路径。
   * @param urdfFile 描述机器人的 URDF 文件的路径。
   * @param referenceFile 参考配置文件的路径。
   * @param useHardFrictionConeConstraint 是否使用硬摩擦锥约束。
   */
  LeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = false);

  ~LeggedInterface() override = default;

  /**
   * @brief 设置最优控制问题。
   * 此函数根据提供的配置文件初始化机器人模型、参考管理器、预计算，并构建代价和约束项。
   * @param taskFile 任务配置文件的路径。
   * @param urdfFile URDF 文件的路径。
   * @param referenceFile 参考配置文件的路径。
   * @param verbose 是否在设置过程中打印详细输出。
   */
  virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose);

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
   * @brief 使用 Pinocchio 设置机器人模型。
   * @param taskFile 任务文件的路径。
   * @param urdfFile URDF 文件的路径。
   * @param referenceFile 参考文件的路径。
   * @param verbose 是否打印详细信息。
   */
  virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  /**
   * @brief 设置用于轨迹跟踪的参考管理器。
   * @param taskFile 任务文件的路径。
   * @param urdfFile URDF 文件的路径。
   * @param referenceFile 参考文件的路径。
   * @param verbose 是否打印详细信息。
   */
  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);

  /**
   * @brief 设置预计算模块。
   * @param taskFile 任务文件的路径。
   * @param urdfFile URDF 文件的路径。
   * @param referenceFile 参考文件的路径。
   * @param verbose 是否打印详细信息。
   */
  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                   bool verbose);

  /**
   * @brief 从文件加载步态计划。
   * @param file 步态文件的路径。
   * @param verbose 是否打印详细信息。
   * @return 指向 GaitSchedule 的共享指针。
   */
  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  /**
   * @brief 创建用于跟踪基座状态的代价项。
   * @param taskFile 任务文件的路径。
   * @param info 质心模型信息。
   * @param verbose 是否打印详细信息。
   * @return 指向用于基座跟踪的 StateInputCost 的唯一指针。
   */
  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);

  /**
   * @brief 初始化输入代价的权重矩阵。
   * @param taskFile 任务文件的路径。
   * @param info 质心模型信息。
   * @return 输入代价权重矩阵。
   */
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  /**
   * @brief 从文件加载摩擦锥设置。
   * @param taskFile 任务文件的路径。
   * @param verbose 是否打印详细信息。
   * @return 包含摩擦系数和惩罚配置的 pair。
   */
  static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose);

  /**
   * @brief 创建一个硬摩擦锥约束。
   * @param contactPointIndex 接触点（脚）的索引。
   * @param frictionCoefficient 摩擦系数。
   * @return 指向 StateInputConstraint 的唯一指针。
   */
  std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);

  /**
   * @brief 创建一个软摩擦锥约束（实现为代价）。
   * @param contactPointIndex 接触点（脚）的索引。
   * @param frictionCoefficient 摩擦系数。
   * @param barrierPenaltyConfig 松弛障碍惩罚的配置。
   * @return 指向 StateInputCost 的唯一指针。
   */
  std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  /**
   * @brief 创建末端执行器运动学模块。
   * @param footNames 脚部框架名称的向量。
   * @param modelName 机器人模型的名称。
   * @return 指向 EndEffectorKinematics 的唯一指针。
   */
  std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& footNames,
                                                                      const std::string& modelName);
  /**
   * @brief 为接触点创建零速度约束。
   * @param eeKinematics 末端执行器运动学。
   * @param contactPointIndex 接触点（脚）的索引。
   * @return 指向 StateInputConstraint 的唯一指针。
   */
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex);
  /**
   * @brief 创建自碰撞避免约束。
   * @param pinocchioInterface 机器人模型的 Pinocchio 接口。
   * @param taskFile 任务文件的路径。
   * @param prefix 加载参数的前缀。
   * @param verbose 是否打印详细信息。
   * @return 指向用于自碰撞的 StateCost 的唯一指针。
   */
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& prefix, bool verbose);

  // 机器人模型和求解器设置
  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  ddp::Settings ddpSettings_;
  sqp::Settings sqpSettings_;
  ipm::Settings ipmSettings_;
  const bool useHardFrictionConeConstraint_;

  // ocs2 接口
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

  // 最优控制问题定义
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  // ocs2 rollout 和 initializer
  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged

#pragma clang diagnostic pop
