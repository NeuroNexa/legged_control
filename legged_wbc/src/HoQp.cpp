//
// Created by qiayuan on 2022/6/28.
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

#include "legged_wbc/HoQp.h"

#include <qpOASES.hpp>
#include <utility>

namespace legged {

// The constructor immediately solves the QP for the current level.
HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  // Initialize variables, pulling data from the higher-priority problem if it exists.
  initVars();
  // Formulate the QP matrices (H, c, D, f) for the current level.
  formulateProblem();
  // Solve the QP using qpOASES.
  solveProblem();
  // Prepare for the next (lower-priority) problem.
  buildZMatrix();          // Compute the null-space projection for the next level.
  stackSlackSolutions();   // Collect slack variable solutions.
}

void HoQp::initVars() {
  // Get properties of the current task.
  numSlackVars_ = task_.d_.rows();
  hasEqConstraints_ = task_.a_.rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  // If there is a higher-priority problem, get its solution and stacked data.
  if (higherProblem_ != nullptr) {
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();
    xPrev_ = higherProblem_->getSolutions(); // This is the particular solution from higher levels.
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();

    // The decision variables for this level live in the null space of the higher levels.
    numDecisionVars_ = stackedZPrev_.cols();
  } else { // This is the highest-priority problem.
    numDecisionVars_ = std::max(task_.a_.cols(), task_.d_.cols());

    // Initialize with empty/identity matrices.
    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
    numPrevSlackVars_ = 0;
  }

  // Combine the current task with all higher-priority tasks.
  stackedTasks_ = task_ + stackedTasksPrev_;
}

void HoQp::formulateProblem() {
  // Build all the matrices required for the QP solver.
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HoQp::buildHMatrix() {
  // The Hessian matrix H represents the cost function: min 0.5 * x' * H * x
  // For equality constraints (Ax=b), the cost is ||Ax - b||^2, which gives H = A'A.
  // We project A into the null space of higher-priority tasks: A_proj = A * Z_prev.
  // So, H = (A_proj)' * (A_proj) = Z_prev' * A' * A * Z_prev.
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    // Make sure that all eigenvalues of A_t_A are non-negative, which could arise due to numerical issues.
    matrix_t aCurrZPrev = task_.a_ * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
  } else {
    zTaTaz.setZero();
  }

  // The full Hessian includes terms for the slack variables, which have a weight of 1.
  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)
            << zTaTaz, matrix_t::Zero(numDecisionVars_, numSlackVars_),
                matrix_t::Zero(numSlackVars_, numDecisionVars_), matrix_t::Identity(numSlackVars_, numSlackVars_))
           .finished();
}

void HoQp::buildCVector() {
  // The vector c represents the linear part of the cost function: min c' * x
  // For equality constraints, this comes from the term -2 * b' * A * x, so c = -A' * b.
  // After substituting x = x_prev + Z_prev * w, the linear term in the new variable w becomes
  // c = Z_prev' * A' * (A * x_prev - b).
  vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    temp = (task_.a_ * stackedZPrev_).transpose() * (task_.a_ * xPrev_ - task_.b_);
  } else {
    temp.setZero();
  }

  // The full c vector also has a zero part for the slack variables.
  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HoQp::buildDMatrix() {
  // The matrix D represents the inequality constraints: D * x <= f.
  // We have two sets of inequalities:
  // 1. Those from higher-priority tasks: D_prev * x <= f_prev
  // 2. Those from the current task: D_curr * x <= f_curr
  // After substituting x = x_prev + Z_prev * w, these become:
  // 1. D_prev * Z_prev * w <= f_prev - D_prev * x_prev
  // 2. D_curr * Z_prev * w <= f_curr - D_curr * x_prev
  matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.d_ * stackedZPrev_;
  } else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);
  }

  // Slack variables (s) are also introduced: D*x - s <= f  and  s >= 0.
  // The matrix here is formulated for the solver which takes D*x <= f.
  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)
            << matrix_t::Zero(numSlackVars_, numDecisionVars_), -matrix_t::Identity(numSlackVars_, numSlackVars_), // -s <= 0  (s >= 0)
                stackedTasksPrev_.d_ * stackedZPrev_, stackedZero,                                                 // D_prev * Z * w <= ...
                dCurrZ,                               -matrix_t::Identity(numSlackVars_, numSlackVars_))          // D_curr * Z * w - s <= ...
           .finished();
}

void HoQp::buildFVector() {
  // The vector f is the right-hand side of the inequality constraints D * x <= f.
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f_ - task_.d_ * xPrev_;
  } else {
    fMinusDXPrev = vector_t::Zero(0);
  }

  // The stacked f vector corresponds to the rows of the D matrix.
  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f_ - stackedTasksPrev_.d_ * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

void HoQp::buildZMatrix() {
  // The null space projection matrix Z is computed from the equality constraints A*x=b.
  // Any solution can be written as x = x_p + Z*w, where x_p is a particular solution
  // and Z is a basis for the null space of A.
  // For the hierarchy, Z_i = Z_{i-1} * Z_curr, where Z_curr is the null space of A_i * Z_{i-1}.
  if (hasEqConstraints_) {
    assert((task_.a_.cols() > 0));
    // We use a robust decomposition method (fullPivLu) to find the kernel (null space).
    stackedZ_ = stackedZPrev_ * (task_.a_ * stackedZPrev_).fullPivLu().kernel();
  } else {
    // If there are no equality constraints, the null space is the entire space.
    stackedZ_ = stackedZPrev_;
  }
}

void HoQp::solveProblem() {
  // Use the qpOASES library to solve the QP.
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC(); // Use settings optimized for MPC applications.
  options.printLevel = qpOASES::PL_LOW; // Suppress most of the solver's output.
  qpProblem.setOptions(options);
  int nWsr = 20; // Number of working set recalculations.

  // Initialize and solve the problem.
  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);

  // Get the solution.
  qpProblem.getPrimalSolution(qpSol.data());

  // Separate the solution into decision variables and slack variables.
  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HoQp::stackSlackSolutions() {
  // Concatenate the slack solutions from this level with those from higher levels.
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

}  // namespace legged
