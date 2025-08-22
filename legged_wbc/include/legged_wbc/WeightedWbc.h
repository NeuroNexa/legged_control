//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

/**
 * @brief 加权全身控制器(Weighted Whole-Body Controller)
 *
 * 这是WBC的另一种具体实现方法。与分层WBC不同，它将所有的任务（包括硬约束和软约束/代价）
 * 合并成一个单一的、大规模的二次规划（QP）问题来求解。
 *
 * 不同的任务通过权重（weight）来区分其重要性。例如，运动学约束会有非常大的权重，
 * 而力矩最小化这类优化目标则会有较小的权重。
 *
 * 这种方法的优点是实现相对简单，并且可以找到一个在所有任务之间折衷的全局最优解。
 * 缺点是它不能像分层WBC那样严格保证高优先级任务的满足。
 */
class WeightedWbc : public WbcBase {
 public:
  // 继承基类的构造函数
  using WbcBase::WbcBase;

  /**
   * @brief 更新函数，实现加权WBC的优化逻辑
   * @param stateDesired MPC给出的期望状态
   * @param inputDesired MPC给出的期望输入
   * @param rbdStateMeasured 测量到的机器人状态
   * @param mode 当前步态模式
   * @param period 控制周期
   * @return 决策变量的优化结果
   */
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  /**
   * @brief 从配置文件加载任务设置，包括权重
   * @param taskFile 任务配置文件路径
   * @param verbose 是否打印详细信息
   */
  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  /**
   * @brief 构建所有的硬约束任务
   * @return 合并后的约束任务
   */
  virtual Task formulateConstraints();

  /**
   * @brief 构建所有的加权软约束/代价任务
   * @param stateDesired 期望状态
   * @param inputDesired 期望输入
   * @param period 控制周期
   * @return 合并并加权后的代价任务
   */
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

 private:
  // 各个代价任务的权重
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged