//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged {
/**
 * @brief WbcBase 构造函数
 */
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  // 决策变量维度 = 广义坐标数 + 接触力维度 + 驱动关节数
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

/**
 * @brief 基类的更新函数
 *
 * 这是一个虚函数，派生类会重写它。
 * 这个基类版本主要负责更新接触状态和模型（测量/期望）。
 */
vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t /*period*/) {
  // 更新接触状态
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  // 更新基于测量状态的Pinocchio模型
  updateMeasured(rbdStateMeasured);
  // 更新基于期望状态的Pinocchio模型
  updateDesired(stateDesired, inputDesired);

  return {};
}

/**
 * @brief 更新基于测量值的模型
 *
 * 使用当前的测量状态（关节位置、速度等）来更新Pinocchio模型，并计算动力学相关量，
 * 如质量矩阵M、非线性效应（科氏力、离心力、重力）C(q,v)、雅可比矩阵J和雅可比导数dJ。
 * 这些量将在构建任务时被使用。
 */
void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  // 从RBD状态向量中提取广义坐标q和广义速度v
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3); // base pos
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();   // base ori
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3); // base linear vel
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum)); // base angular vel
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // 更新运动学量
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  // 更新动力学量
  pinocchio::crba(model, data, qMeasured_); // 质量矩阵 M
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_); // 科氏力、离心力、重力 nle=C(q,v)v+g
  // 计算足端雅可比
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    matrix_t jac(6, info_.generalizedCoordinatesNum);
    jac.setZero();
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // 计算足端雅可比的时间导数
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    matrix_t jac(6, info_.generalizedCoordinatesNum);
    jac.setZero();
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
}

/**
 * @brief 更新基于期望值的模型
 */
void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  // ... (与updateMeasured类似，但使用期望状态和输入来更新另一个Pinocchio模型实例)
}

/**
 * @brief 构建浮动基座的运动方程任务 (硬约束)
 *
 * M(q)\ddot{q} + C(q,\dot{q}) = S^T\tau + J_c^T F_c
 * 移项整理成 Ax = b 的形式:
 * [M, -J_c^T, -S^T] * [\ddot{q}, F_c, \tau]^T = -C(q,\dot{q})
 */
Task WbcBase::formulateFloatingBaseEomTask() {
  auto& data = pinocchioInterfaceMeasured_.getData();
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity(); // 关节力矩的选择矩阵

  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

/**
 * @brief 构建关节力矩限制任务 (不等式约束)
 *
 * -\tau_limit <= \tau <= \tau_limit
 * 整理成 Dx <= f 的形式:
 * [ 0, 0,  I] * x <=  \tau_limit
 * [ 0, 0, -I] * x <=  \tau_limit
 */
Task WbcBase::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  f.head(info_.actuatedDofNum) = torqueLimits_;
  f.tail(info_.actuatedDofNum) = torqueLimits_;

  return {matrix_t(), vector_t(), d, f};
}

/**
 * @brief 构建支撑足的运动约束 (硬约束)
 *
 * 对于支撑足，其加速度应为0: a_c = \dot{J}\dot{q} + J\ddot{q} = 0
 * 整理成 Ax = b 的形式:
 * [J_c, 0, 0] * x = -\dot{J}_c\dot{q}
 */
Task WbcBase::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }
  return {a, b, matrix_t(), vector_t()};
}

/**
 * @brief 构建摩擦锥约束 (不等式约束)
 *
 * 对于支撑足，接触力需要满足摩擦锥约束。
 * 对于摆动足，接触力需要为0。
 */
Task WbcBase::formulateFrictionConeTask() {
  // ... (构建矩阵D和向量f)
  return {a, b, d, f};
}

/**
 * @brief 构建基座加速度跟踪任务 (代价)
 *
 * 最小化 || \ddot{q}_{base} - \ddot{q}_{base,des} ||^2
 */
Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  // ... (计算期望的基座加速度 b)
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);
  return {a, b, matrix_t(), vector_t()};
}

/**
 * @brief 构建摆动腿加速跟踪任务 (代价)
 *
 * 最小化 || a_{swing} - a_{swing,des} ||^2
 * 其中期望加速度 a_{swing,des} 通过PD控制计算: Kp(p_des - p) + Kd(v_des - v)
 */
Task WbcBase::formulateSwingLegTask() {
  // ... (计算期望的摆动腿加速度 b)
  return {a, b, matrix_t(), vector_t()};
}

/**
 * @brief 构建接触力跟踪任务 (代价)
 *
 * 最小化 || F_c - F_{c,des} ||^2
 */
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  // ... (构建矩阵A和向量b)
  return {a, b, matrix_t(), vector_t()};
}

/**
 * @brief 从配置文件加载任务参数
 */
void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // ... (加载力矩限制、摩擦系数、PD增益等)
}

}  // namespace legged
