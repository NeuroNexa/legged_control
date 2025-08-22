/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_legged_robot/common/Types.h>

#include "legged_interface/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 摩擦锥约束
 *
 * 实现了不等式约束 h(t,x,u) >= 0，用于确保足端的接触力在摩擦锥内部。
 *
 * 具体的约束形式为:
 * frictionCoefficient * (Fz + gripperForce) - sqrt(Fx^2 + Fy^2 + regularization) >= 0
 *
 * 其中 Fx, Fy, Fz 是在地形局部坐标系下的接触力。
 *
 * - gripperForce: 抓取力，可以使摩擦锥的原点在法向（Z轴）上向下移动，
 *   允许在没有法向力的情况下产生切向力，或者“拉”地面。
 * - regularization: 正则化项，用于防止当Fx=Fy=Fz=0时，约束的梯度/Hessian矩阵变为无穷大。
 *   它也相当于在摩擦锥边界上创建了一个抛物线的安全裕度。
 */
class FrictionConeConstraint final : public StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 摩擦锥约束的配置
   */
  struct Config {
    explicit Config(scalar_t frictionCoefficientParam = 0.7, scalar_t regularizationParam = 25.0, scalar_t gripperForceParam = 0.0,
                    scalar_t hessianDiagonalShiftParam = 1e-6)
        : frictionCoefficient(frictionCoefficientParam),
          regularization(regularizationParam),
          gripperForce(gripperForceParam),
          hessianDiagonalShift(hessianDiagonalShiftParam) {
      assert(frictionCoefficient > 0.0);
      assert(regularization > 0.0);
      assert(hessianDiagonalShift >= 0.0);
    }

    scalar_t frictionCoefficient; // 摩擦系数
    scalar_t regularization;    // 正则化项
    scalar_t gripperForce;      // 抓取力
    scalar_t hessianDiagonalShift; // Hessian对角线偏移，确保二次逼近是凸的
  };

  /**
   * @brief 构造函数
   * @param referenceManager 参考管理器，用于查询步态信息。
   * @param config 摩擦锥模型的设置。
   * @param contactPointIndex 接触点的索引。
   * @param info 质心模型信息。
   */
  FrictionConeConstraint(const SwitchedModelReferenceManager& referenceManager, Config config, size_t contactPointIndex,
                         CentroidalModelInfo info);

  ~FrictionConeConstraint() override = default;
  FrictionConeConstraint* clone() const override { return new FrictionConeConstraint(*this); }

  // --- OCS2 StateInputConstraint 接口的实现 ---
  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; };
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

  /** 设置地形法向量（在世界坐标系下表示） */
  void setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld);

 private:
  // --- 用于计算导数的辅助数据结构和函数 ---
  struct LocalForceDerivatives { /* ... */ };
  struct ConeLocalDerivatives { /* ... */ };
  struct ConeDerivatives { /* ... */ };

  FrictionConeConstraint(const FrictionConeConstraint& other) = default;
  vector_t coneConstraint(const vector3_t& localForces) const;
  LocalForceDerivatives computeLocalForceDerivatives(const vector3_t& forcesInBodyFrame) const;
  ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t& localForces) const;
  ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const;
  matrix_t frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const;
  matrix_t frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const;

  // --- 成员变量 ---
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const Config config_;
  const size_t contactPointIndex_;
  const CentroidalModelInfo info_;

  // 从世界坐标系到地形局部坐标系的旋转矩阵
  matrix3_t t_R_w = matrix3_t::Identity();
};

}  // namespace legged_robot
}  // namespace ocs2
