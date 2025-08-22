//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

/**
 * @class HierarchicalWbc
 * @brief 一种使用任务层次结构解决WBC问题的实现。
 *
 * 此类继承自 WbcBase 并实现了 `update` 方法。在 `update` 方法内部，
 * 它构建控制任务，并使用级联的二次规划（QP）按优先级顺序解决这些任务。
 * 较高优先级QP的解将成为后续较低优先级QP的约束。
 *
 * 层次化优化的实际实现由 HoQp 类处理。
 */
class HierarchicalWbc : public WbcBase {
 public:
  // 继承基类的构造函数。
  using WbcBase::WbcBase;

  /**
   * @brief 解决分层的全身控制问题。
   *
   * 此方法覆盖基类的 update 方法。它构建任务，然后
   * 按顺序解决它们，以找到最优的决策变量。
   *
   * @param stateDesired 来自 MPC 的期望状态。
   * @param inputDesired 来自 MPC 的期望输入。
   * @param rbdStateMeasured 测量的机器人状态。
   * @param mode 当前的接触模式。
   * @param period 控制周期。
   * @return 计算出的最优决策变量（加速度、力、力矩）。
   */
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
};

}  // namespace legged
