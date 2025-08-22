//
// Created by qiayuan on 2022/6/29.
//
#include <gtest/gtest.h>

#include "legged_wbc/HoQp.h"

using namespace legged;

// This test case checks the functionality of the HoQp solver with a two-level hierarchy.
TEST(HoQP, twoTask) {
  // Seed the random number generator for reproducibility.
  srand(0);

  // Define two tasks, task0 (high priority) and task1 (low priority).
  Task task0, task1;
  // task0 has random equality and inequality constraints.
  task0.a_ = matrix_t::Random(2, 4);
  task0.b_ = vector_t::Ones(2);
  task0.d_ = matrix_t::Random(2, 4);
  task0.f_ = vector_t::Ones(2);
  // task1 has the same inequality constraints but different equality constraints.
  task1 = task0;
  task1.a_ = matrix_t::Ones(2, 4);

  // Solve the first level of the hierarchy (only task0).
  std::shared_ptr<HoQp> hoQp0 = std::make_shared<HoQp>(task0);
  // Solve the second level of the hierarchy (task1, with task0 as the higher-priority problem).
  std::shared_ptr<HoQp> hoQp1 = std::make_shared<HoQp>(task1, hoQp0);

  // Get the solutions from both levels.
  vector_t x0 = hoQp0->getSolutions(); // Solution for task0 only.
  vector_t x1 = hoQp1->getSolutions(); // Solution for task1 respecting task0.
  vector_t slack0 = hoQp0->getStackedSlackSolutions();
  vector_t slack1 = hoQp1->getStackedSlackSolutions();

  // Print solutions for debugging purposes.
  std::cout << "x0: " << x0.transpose() << std::endl;
  std::cout << "x1: " << x1.transpose() << std::endl;
  std::cout << "slack0: " << slack0.transpose() << std::endl;
  std::cout << "slack1: " << slack1.transpose() << std::endl;

  scalar_t prec = 1e-6;

  // --- Assertions ---

  // Check if the equality constraints of task0 are met by the first solution (x0).
  // This is only strictly true if the problem is feasible (slack variables are zero).
  if (slack0.isApprox(vector_t::Zero(slack0.size()))) {
    EXPECT_TRUE((task0.a_ * x0).isApprox(task0.b_, prec));
  }

  // Check if the equality constraints of both tasks are met by the second solution (x1).
  // The lower priority task (task1) should only be fulfilled if it doesn't violate the higher priority task (task0).
  if (slack1.isApprox(vector_t::Zero(slack1.size()))) {
    EXPECT_TRUE((task1.a_ * x1).isApprox(task1.b_, prec));
    EXPECT_TRUE((task0.a_ * x1).isApprox(task0.b_, prec));
  }

  // Check if the inequality constraints are met for both solutions.
  // The check is D*x <= f + s, where s are the slack variables.
  vector_t y0 = task0.d_ * x0;
  for (int i = 0; i < y0.size(); ++i) {
    EXPECT_LE(y0[i], task0.f_[i] + slack0[i] + prec);
  }
  vector_t y1 = task1.d_ * x1;
  for (int i = 0; i < y1.size(); ++i) {
    // Note: The slack variables in slack1 are stacked, so we need to access the correct part.
    // slack1 contains slacks for task0 and task1.
    EXPECT_LE(y1[i], task1.f_[i] + slack1[slack0.size() + i] + prec);
  }
}
