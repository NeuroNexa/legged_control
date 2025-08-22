//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>

#include "legged_wbc/HoQp.h"

using namespace legged;

/**
 * @brief GTest测试用例，用于测试HoQp求解器
 *
 * 这个测试用例构建了一个包含两个层级的简单分层QP问题，
 * 并检查求解器得到的解是否满足两个层级的约束。
 */
TEST(HoQP, twoTask) {
  // 使用固定的随机种子，确保测试结果可复现
  srand(0);

  // --- 创建两个任务 ---
  // task0 是高优先级任务，task1 是低优先级任务
  Task task0, task1;
  // 随机生成task0的矩阵和向量
  task0.a_ = matrix_t::Random(2, 4); // 等式约束/代价
  task0.b_ = vector_t::Ones(2);
  task0.d_ = matrix_t::Random(2, 4); // 不等式约束
  task0.f_ = vector_t::Ones(2);
  // task1基于task0，但使用不同的代价矩阵
  task1 = task0;
  task1.a_ = matrix_t::Ones(2, 4);

  // --- 求解分层QP ---
  // 1. 先求解高优先级问题
  std::shared_ptr<HoQp> hoQp0 = std::make_shared<HoQp>(task0);
  // 2. 然后在hoQp0的基础上求解低优先级问题
  std::shared_ptr<HoQp> hoQp1 = std::make_shared<HoQp>(task1, hoQp0);

  // 获取解
  vector_t x0 = hoQp0->getSolutions(); // 只求解task0时的解
  vector_t x1 = hoQp1->getSolutions(); // 求解task0+task1时的解
  vector_t slack0 = hoQp0->getStackedSlackSolutions(); // task0的松弛变量
  vector_t slack1 = hoQp1->getStackedSlackSolutions(); // task0+task1的松弛变量

  // 打印解以供调试
  std::cout << "Solution for task0: " << x0.transpose() << std::endl;
  std::cout << "Solution for task0+1: " << x1.transpose() << std::endl;
  std::cout << "Slack for task0: " << slack0.transpose() << std::endl;
  std::cout << "Slack for task0+1: " << slack1.transpose() << std::endl;

  scalar_t prec = 1e-6; // 精度

  // --- 检查解的有效性 ---
  // 检查task0的等式约束是否满足
  // 如果松弛变量为0，那么等式约束应该被严格满足
  if (slack0.isApprox(vector_t::Zero(slack0.size()))) {
    EXPECT_TRUE((task0.a_ * x0).isApprox(task0.b_, prec));
  }

  // 检查task1的等式约束是否满足
  if (slack1.isApprox(vector_t::Zero(slack1.size()))) {
    // 检查task1的解是否满足task1的约束
    EXPECT_TRUE((task1.a_ * x1).isApprox(task1.b_, prec));
    // 关键：检查task1的解是否仍然满足更高优先级的task0的约束
    EXPECT_TRUE((task0.a_ * x1).isApprox(task0.b_, prec));
  }

  // 检查不等式约束 D*x <= f + slack
  vector_t y = task0.d_ * x0;
  for (int i = 0; i < y.size(); ++i) {
    EXPECT_TRUE(y[i] <= task0.f_[i] + slack0[i] + prec);
  }
  y = task1.d_ * x1;
  for (int i = 0; i < y.size(); ++i) {
    EXPECT_TRUE(y[i] <= task1.f_[i] + slack1[i] + prec);
  }
}
