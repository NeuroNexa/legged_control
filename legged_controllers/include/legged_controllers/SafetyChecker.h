//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged {
using namespace ocs2;
using namespace centroidal_model;

/**
 * @class SafetyChecker
 * @brief 一个用于检查机器人状态是否在安全操作范围内的类。
 *
 * 此类负责对机器人的当前和计划状态执行安全检查。
 * 如果安全检查失败，它可以触发控制器关闭，以防止机器人损坏。
 * 目前，它只实现了一个对机器人方向（俯仰角）的基本检查。
 * 未来可以扩展以包括其他检查，例如关节位置/速度限制、
 * 力矩限制或碰撞检测。
 */
class SafetyChecker {
 public:
  /**
   * @brief SafetyChecker 的构造函数。
   * @param info 质心模型信息。
   */
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  /**
   * @brief 主安全检查函数。
   * @param observation 当前的系统观测值（状态、输入等）。
   * @param optimized_state 来自 MPC 的计划最优状态。
   * @param optimized_input 来自 MPC 的计划最优输入。
   * @return 如果所有安全检查都通过，则为 true，否则为 false。
   */
  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    // 目前，只对当前观测值执行方向检查。
    return checkOrientation(observation);
  }

 protected:
  /**
   * @brief 检查机器人的方向是否在安全限制内。
   * @param observation 当前的系统观测值。
   * @return 如果方向安全，则为 true，否则为 false。
   */
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    // 检查俯仰角（由基座姿态的第6个元素近似，可能是ZYX欧拉角）是否在[-pi/2, pi/2]范围内。
    if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
      std::cerr << "[SafetyChecker] 方向安全检查失败！俯仰角过大。" << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace legged
