/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#include "legged_interface/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief EndEffectorLinearConstraint 构造函数
 */
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t numConstraints, Config config)
    : StateInputConstraint(ConstraintOrder::Linear), // 声明这是一个线性约束
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      numConstraints_(numConstraints),
      config_(std::move(config)) {
  // 确保这个约束只用于单个末端执行器
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief EndEffectorLinearConstraint 拷贝构造函数
 */
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs)
    : StateInputConstraint(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 配置新的约束系数
 */
void EndEffectorLinearConstraint::configure(Config&& config) {
  // 断言检查，确保传入的配置维度正确
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
  config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的值 g(x, u)
 *
 * g(x, u) = A_x * x_ee(x) + A_v * v_ee(x, u) + b
 */
vector_t EndEffectorLinearConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  vector_t f = config_.b;
  // 如果定义了位置约束部分
  if (config_.Ax.size() > 0) {
    // 计算末端执行器位置 x_ee，并乘以 A_x
    f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
  }
  // 如果定义了速度约束部分
  if (config_.Av.size() > 0) {
    // 计算末端执行器速度 v_ee，并乘以 A_v
    f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front();
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 计算约束的线性逼近
 *
 * 返回 g(x,u) 的值以及其对于状态x和输入u的雅可比矩阵 (偏导数)。
 * df/dx = A_x * dx_ee/dx + A_v * dv_ee/dx
 * df/du = A_v * dv_ee/du
 */
VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  // 初始化一个空的线性逼近对象
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = config_.b;

  // 如果定义了位置约束部分
  if (config_.Ax.size() > 0) {
    // 获取末端执行器位置的线性逼近 (即 x_ee 的值和雅可比矩阵 dx_ee/dx)
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
    linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
  }

  // 如果定义了速度约束部分
  if (config_.Av.size() > 0) {
    // 获取末端执行器速度的线性逼近 (即 v_ee 的值和雅可比矩阵 dv_ee/dx, dv_ee/du)
    const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
    linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
    linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
    linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
