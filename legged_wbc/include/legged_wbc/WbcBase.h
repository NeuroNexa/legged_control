//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @class WbcBase
 * @brief 全身控制器（WBC）的基类。
 *
 * WBC 计算最优的关节力矩和接触力，以实现期望的运动，
 * 同时遵守各种物理和任务空间约束。它将控制问题
 * 表述为一组有优先级的任务（约束和目标），然后由派生类
 * （例如 HierarchicalWbc 或 WeightedWbc）求解。
 *
 * 优化问题的决策变量是：
 *   x = [广义加速度^T, 接触力^T, 关节力矩^T]^T
 */
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  /**
   * @brief WbcBase 的构造函数。
   * @param pinocchioInterface 机器人的 Pinocchio 接口。
   * @param info 质心模型信息。
   * @param eeKinematics 末端执行器运动学。
   */
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief 从配置文件加载任务特定设置。
   * @param taskFile 任务配置文件的路径。
   * @param verbose 是否打印加载的设置。
   */
  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  /**
   * @brief WBC 的主更新循环。
   * @param stateDesired 来自 MPC 的期望状态。
   * @param inputDesired 来自 MPC 的期望输入。
   * @param rbdStateMeasured 测量的机器人状态（广义坐标和速度）。
   * @param mode 当前的接触模式。
   * @param period 控制周期。
   * @return 计算出的最优决策变量（加速度、力、力矩）。
   */
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  /**
   * @brief 用测量的机器人状态更新内部模型。
   * @param rbdStateMeasured 测量的机器人状态。
   */
  void updateMeasured(const vector_t& rbdStateMeasured);

  /**
   * @brief 用期望的状态和输入更新内部模型。
   * @param stateDesired 期望状态。
   * @param inputDesired 期望输入。
   */
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  // 任务构建方法
  Task formulateFloatingBaseEomTask();        //!< 构建浮动基座运动方程任务。
  Task formulateTorqueLimitsTask();           //!< 构建关节力矩限制任务。
  Task formulateNoContactMotionTask();        //!< 构建摆动脚无约束运动的任务。
  Task formulateFrictionConeTask();           //!< 构建支撑脚的摩擦锥约束任务。
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);  //!< 构建基座加速度跟踪任务。
  Task formulateSwingLegTask();               //!< 构建摆动腿运动跟踪任务。
  Task formulateContactForceTask(const vector_t& inputDesired) const;  //!< 构建接触力跟踪任务。

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  // 每个周期更新的内部状态变量
  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;  // 接触雅可比矩阵及其时间导数
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // 从配置文件加载的任务参数
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace legged
