//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged {
using namespace ocs2;
using namespace centroidal_model;

/**
 * @brief 安全检查器
 *
 * 该类用于检查机器人的状态和将要执行的动作是否安全。
 * 如果检测到不安全的状况（例如，机器人即将翻倒），它可以触发相应的保护措施。
 */
class SafetyChecker {
 public:
  /**
   * @brief 构造函数
   * @param info 质心模型信息，用于解析状态向量
   */
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  /**
   * @brief 执行安全检查
   * @param observation 当前的系统观测值
   * @param optimized_state MPC优化后的状态（当前未使用）
   * @param optimized_input MPC优化后的输入（当前未使用）
   * @return 如果检查通过则为 true，否则为 false
   */
  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    // 目前只实现了姿态检查
    return checkOrientation(observation);
  }

 protected:
  /**
   * @brief 检查机器人的姿态是否在安全范围内
   * @param observation 当前的系统观测值
   * @return 如果姿态安全则为 true
   */
  bool checkOrientation(const SystemObservation& observation) {
    // 从观测状态中获取基座的姿态
    vector_t pose = getBasePose(observation.state, info_);

    // 检查roll角（pose(5)）是否超出了 +/- 90度
    // 如果超出，意味着机器人可能已经翻倒或即将翻倒
    if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_; // 质心模型信息的引用
};

}  // namespace legged
