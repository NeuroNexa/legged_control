//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged {

/**
 * @brief 更新加权WBC
 *
 * @param stateDesired MPC给出的期望状态
 * @param inputDesired MPC给出的期望输入
 * @param rbdStateMeasured 测量到的机器人状态
 * @param mode 当前步态模式
 * @param period 控制周期
 * @return 决策变量的优化结果
 */
vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  // 调用基类的方法来更新模型、雅可比等
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // --- 构建QP问题 ---
  // QP问题形式:
  // min 1/2 * x'Hx + g'x
  // s.t. lbA <= Ax <= ubA

  // 1. 构建约束矩阵 A 和约束边界 lbA, ubA
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);
  // Ax = b  (等式约束)
  A.topRows(constraints.a_.rows()) = constraints.a_;
  lbA.head(constraints.b_.size()) = constraints.b_;
  ubA.head(constraints.b_.size()) = constraints.b_;
  // Dx <= f (不等式约束)
  A.bottomRows(constraints.d_.rows()) = constraints.d_;
  lbA.tail(constraints.f_.size()) = -qpOASES::INFTY * vector_t::Ones(constraints.f_.size()); // 下界为负无穷
  ubA.tail(constraints.f_.size()) = constraints.f_;

  // 2. 构建代价函数矩阵 H 和 g
  // 将所有代价任务加权并合并
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  // H = A_cost' * A_cost
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  // g = -A_cost' * b_cost
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // 3. 使用qpOASES求解QP问题
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 20; // 最大迭代次数

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());
  qpProblem.getPrimalSolution(qpSol.data());

  return qpSol;
}

/**
 * @brief 构建所有的硬约束任务
 */
Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

/**
 * @brief 构建所有的加权软约束/代价任务
 */
Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ +
         formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

/**
 * @brief 从配置文件加载任务权重
 */
void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose); // 调用基类方法加载通用参数

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
