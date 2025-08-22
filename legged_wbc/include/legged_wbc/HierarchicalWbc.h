//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

/**
 * @brief 分层全身控制器(Hierarchical Whole-Body Controller)
 *
 * 这是一种WBC的具体实现方法。它将控制问题分解为一系列按优先级排序的子问题。
 * 首先，它会满足最高优先级的任务（通常是物理约束，如运动学和动力学方程）。
 * 然后，在满足高优先级任务的解空间（零空间）内，它会去优化次高优先级的任务，以此类推。
 *
 * 这种方法可以严格保证高优先级任务的满足，但可能会牺牲低优先级任务的性能。
 */
class HierarchicalWbc : public WbcBase {
 public:
  // 继承基类的构造函数
  using WbcBase::WbcBase;

  /**
   * @brief 更新函数，实现分层WBC的优化逻辑
   * @param stateDesired MPC给出的期望状态
   * @param inputDesired MPC给出的期望输入
   * @param rbdStateMeasured 测量到的机器人状态
   * @param mode 当前步态模式
   * @param period 控制周期
   * @return 决策变量的优化结果
   */
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
};

}  // namespace legged
