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
 * @brief 全身控制器(WBC)的基类
 *
 * WBC的目标是根据MPC给出的期望状态和输入（通常来自简化的质心动力学模型），
 * 结合完整的刚体动力学模型，计算出能够实现该运动的关节力矩。
 *
 * 这个问题通常被构建为一个带约束的优化问题（通常是二次规划QP）。
 *
 * 决策变量 (Decision Variables): x = [\ddot{q}^T, F_c^T, \tau^T]^T
 * - \ddot{q}: 广义加速度 (包括基座和关节)
 * - F_c: 接触力
 * - \tau: 关节力矩
 *
 * 不同的WBC实现（如分层WBC、加权WBC）会以不同的方式来求解这个优化问题。
 * 这个基类提供了构建该优化问题所需的所有任务（约束和代价）。
 */
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  /**
   * @brief 构造函数
   * @param pinocchioInterface Pinocchio模型接口
   * @param info 质心模型信息
   * @param eeKinematics 末端执行器运动学
   */
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  /**
   * @brief 从配置文件加载任务设置
   * @param taskFile 任务配置文件路径
   * @param verbose 是否打印详细信息
   */
  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  /**
   * @brief 主更新函数，由派生类实现
   * @param stateDesired MPC给出的期望状态
   * @param inputDesired MPC给出的期望输入
   * @param rbdStateMeasured 测量到的机器人状态（刚体动力学模型格式）
   * @param mode 当前步态模式
   * @param period 控制周期
   * @return 决策变量的优化结果
   */
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period) = 0; // 纯虚函数，由派生类实现

 protected:
  /**
   * @brief 更新测量相关的模型数据（如雅可比等）
   * @param rbdStateMeasured 测量到的机器人状态
   */
  void updateMeasured(const vector_t& rbdStateMeasured);

  /**
   * @brief 更新期望相关的模型数据
   * @param stateDesired 期望状态
   * @param inputDesired 期望输入
   */
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  // --- 用于构建优化问题的任务 ---
  // 每个函数返回一个Task对象，该对象包含了约束或代价的矩阵形式 (Ax=b, Gx<=h, or 1/2*x'Hx+g'x)

  Task formulateFloatingBaseEomTask(); // 浮动基座的运动方程 (硬约束)
  Task formulateTorqueLimitsTask(); // 关节力矩限制 (不等式约束)
  Task formulateNoContactMotionTask(); // 非接触腿的运动约束 (硬约束)
  Task formulateFrictionConeTask(); // 摩擦锥约束 (不等式约束)
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period); // 基座加速度跟踪任务 (代价)
  Task formulateSwingLegTask(); // 摆动腿运动任务 (代价)
  Task formulateContactForceTask(const vector_t& inputDesired) const; // 接触力跟踪任务 (代价)

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_; // 雅可比矩阵和其导数
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // 任务参数
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace legged
