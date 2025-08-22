/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 腿式机器人的初始化器
 *
 * 该类继承自 `Initializer`，负责为最优控制求解器提供一个初始的“猜测”轨迹。
 * 当求解器开始一次新的优化时（例如，在MPC的每个时间步），它需要一个初始的解（状态和输入轨迹）作为起点。
 *
 * 这个初始化器会根据当前的机器人状态和参考管理器中的目标轨迹，
 * 来计算出一个合理的初始输入 `u(t)` 和下一个状态 `x(t+dt)`。
 * 一个好的初始猜测可以显著提高求解器的收敛速度和稳定性。
 */
class LeggedRobotInitializer final : public Initializer {
 public:
  /**
   * @brief 构造函数
   * @param info 质心模型信息。
   * @param referenceManager 切换系统参考管理器。
   * @param extendNormalizedMomentum 如果为true，则外推归一化动量；否则将其设为零。
   */
  LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                         bool extendNormalizedMomentum = false);

  ~LeggedRobotInitializer() override = default;
  LeggedRobotInitializer* clone() const override;

  /**
   * @brief 计算初始的输入和下一个状态
   * @param time      [in]  当前时间
   * @param state     [in]  当前状态
   * @param nextTime  [in]  下一个时间点
   * @param input     [out] 计算出的初始输入
   * @param nextState [out] 计算出的下一个状态
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  LeggedRobotInitializer(const LeggedRobotInitializer& other) = default;

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const bool extendNormalizedMomentum_;
};

}  // namespace legged_robot
}  // namespace ocs2
