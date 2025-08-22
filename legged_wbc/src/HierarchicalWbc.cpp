//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  // 更新基类，计算运动学和动力学量
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // 构建任务并按优先级分组。
  // 任务的 `+` 运算符将它们合并为单个任务。

  // 任务0：最高优先级。此组包括基本的物理约束。
  // - 浮动基座运动方程：必须满足。
  // - 力矩限制：执行器的物理限制。
  // - 摩擦锥：防止脚部打滑。
  // - 无接触运动：确保支撑脚保持在地面上。
  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();

  // 任务1：中等优先级。此组处理运动跟踪。
  // - 基座加速度：跟踪来自 MPC 的期望基座运动。
  // - 摆动腿：跟踪期望的摆动脚轨迹。
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();

  // 任务2：最低优先级。此组处理期望力的跟踪。
  // - 接触力：尝试实现来自 MPC 的期望接触力。这通常是一个“软”目标。
  Task task2 = formulateContactForceTask(inputDesired);

  // 使用 HoQp (Hierarchical Optimization for QP) 构建二次规划（QP）的级联。
  // QP 是嵌套的，因此较高优先级 QP 的解将成为较低优先级 QP 的约束。
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  // 解决分层 QP 并返回解。
  return hoQp.getSolutions();
}

}  // namespace legged
