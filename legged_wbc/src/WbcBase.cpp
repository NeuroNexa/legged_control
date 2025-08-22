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
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  // Total number of decision variables in the optimization problem
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t /*period*/) {
  // Update contact flags and number of contacts
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  // Update the internal model with measured and desired states
  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  // Base class returns an empty vector. Derived classes will solve the WBC problem and return the result.
  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  // Extract generalized coordinates and velocities from the measured state vector
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // Perform forward kinematics and compute dynamics quantities for the measured state
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);  // Composite Rigid Body Algorithm (CRBA) for mass matrix
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);  // Coriolis, centrifugal, and gravity terms

  // Compute the contact Jacobian (J)
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // Compute the time derivative of the contact Jacobian (dJ/dt)
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  // Perform forward kinematics for the desired state to be used in task formulations
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);
}

Task WbcBase::formulateFloatingBaseEomTask() {
  // This task enforces the robot's equations of motion: M*a + h = S*tau + J^T*f
  // where a is generalized acceleration, h is nonlinear effects, tau is joint torques, f is contact forces.
  // It is formulated as an equality constraint: [M, -J^T, -S^T] * [a; f; tau] = -h
  auto& data = pinocchioInterfaceMeasured_.getData();

  // Selection matrix for actuated DOFs
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateTorqueLimitsTask() {
  // This task enforces torque limits: -limit <= tau <= limit
  // It is formulated as an inequality constraint: [0, 0, I; 0, 0, -I] * [a; f; tau] <= [limit; limit]
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  // tau <= limit
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  // -tau <= limit  (or tau >= -limit)
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
    f.segment<3>(3 * l) = torqueLimits_;
  }

  return {matrix_t(), vector_t(), d, f};
}

Task WbcBase::formulateNoContactMotionTask() {
  // This task enforces the zero acceleration constraint for feet in contact (J*a + dJ*v = 0).
  // This is a prerequisite for formulating the friction cone constraint.
  // It is formulated as an equality constraint for all stance feet.
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

Task WbcBase::formulateFrictionConeTask() {
  // This task combines two things:
  // 1. Zero force constraint for swing feet (equality).
  // 2. Friction cone constraint for stance feet (inequality).
  // It uses a linearized friction pyramid for the inequality constraint.

  // Part 1: Zero force for swing feet
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  // Part 2: Friction cone for stance feet (linearized pyramid)
  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,                // Fz <= 0 (normal force must be downwards)
                     1, 0, -frictionCoeff_,   // Fx <= mu * Fz
                    -1, 0, -frictionCoeff_,   // -Fx <= mu * Fz
                     0, 1, -frictionCoeff_,   // Fy <= mu * Fz
                     0,-1, -frictionCoeff_;   // -Fy <= mu * Fz
                     // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  // This task tracks the desired base acceleration from the MPC.
  // It is formulated as an equality constraint on the first 6 decision variables (base acceleration).
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  // Calculate the desired base acceleration based on the centroidal dynamics
  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateSwingLegTask() {
  // This task tracks a desired acceleration for the swing legs using a PD controller in Cartesian space.
  // The desired acceleration is computed to drive the swing foot towards its desired trajectory.
  // Formulated as an equality constraint: J*a + dJ*v = acc_desired

  // Get measured and desired end-effector positions and velocities
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      // PD control to calculate desired acceleration
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      // Set up the equality constraint matrix
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  // This task tracks the desired contact forces from the MPC.
  // It's typically a lower-priority task.
  // Formulated as an equality constraint on the force decision variables.
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
}

}  // namespace legged
