//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
/**
 * @brief 更新分层WBC
 *
 * @param stateDesired MPC给出的期望状态
 * @param inputDesired MPC给出的期望输入
 * @param rbdStateMeasured 测量到的机器人状态
 * @param mode 当前步态模式
 * @param period 控制周期
 * @return 决策变量的优化结果
 */
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  // 调用基类的方法来更新模型、雅可比等
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // --- 构建任务层级 ---
  // 优先级从高到低： task0 > task1 > task2

  // 任务0 (最高优先级): 物理约束
  // 包括：浮动基座动力学、力矩限制、摩擦锥、支撑腿运动学约束
  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();

  // 任务1 (次高优先级): 运动任务
  // 包括：基座加速度跟踪、摆动腿轨迹跟踪
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();

  // 任务2 (最低优先级): 优化目标
  // 包括：接触力跟踪（最小化接触力）
  Task task2 = formulateContactForceTask(inputDesired);

  // --- 求解分层QP ---
  // 通过嵌套构造HoQp对象来创建和求解分层问题。
  // task0是最高优先级，所以它在最内层。
  // HoQp的解会自动在更高优先级的任务的零空间内进行优化。
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  // 返回最终的解
  return hoQp.getSolutions();
}

}  // namespace legged
