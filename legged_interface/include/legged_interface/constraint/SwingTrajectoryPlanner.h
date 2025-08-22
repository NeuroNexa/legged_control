/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
... (license header)
******************************************************************************/

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>

namespace ocs2 {
namespace legged_robot {

/**
 * @brief 摆动腿轨迹规划器
 *
 * 该类根据给定的步态序列（ModeSchedule），为每个处于摆动相的腿生成一个平滑的垂直（Z轴）轨迹。
 * 这个轨迹通常是一个三次样条曲线，定义了摆动腿从抬起到落地的整个过程中的期望高度和垂直速度。
 * 生成的轨迹随后被用作`NormalVelocityConstraint`等约束的目标值。
 */
class SwingTrajectoryPlanner {
 public:
  /**
   * @brief 配置结构体
   */
  struct Config {
    scalar_t liftOffVelocity = 0.0;     // 抬腿时的目标垂直速度
    scalar_t touchDownVelocity = 0.0;   // 落足时的目标垂直速度
    scalar_t swingHeight = 0.1;         // 摆动相的最高点高度
    scalar_t swingTimeScale = 0.15;     // 时间缩放因子。如果摆动相比这个时间短，摆动高度也会相应缩放。
  };

  /**
   * @brief 构造函数
   * @param config 规划器的配置
   * @param numFeet 足的数量
   */
  SwingTrajectoryPlanner(Config config, size_t numFeet);

  /**
   * @brief 更新规划器
   *
   * 根据新的步态序列和地形高度，重新生成所有腿的摆动轨迹。
   * @param modeSchedule 步态序列
   * @param terrainHeight 地形高度
   */
  void update(const ModeSchedule& modeSchedule, scalar_t terrainHeight);

  /**
   * @brief 更新规划器的重载版本
   *
   * 允许为每个抬起和落下事件指定不同的地形高度。
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence);

  /**
   * @brief 更新规划器的重载版本
   *
   * 允许为每个摆动相指定不同的最大摆动高度。
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence, const feet_array_t<scalar_array_t>& maxHeightSequence);

  /**
   * @brief 获取指定腿在指定时间的期望垂直速度
   * @param leg 腿的索引
   * @param time 时间
   * @return 期望的Z轴速度
   */
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  /**
   * @brief 获取指定腿在指定时间的期望垂直位置
   * @param leg 腿的索引
   * @param time 时间
   * @return 期望的Z轴位置
   */
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

 private:
  /**
   * @brief 从模式ID序列中提取接触标志序列
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * @brief 查找指定腿的抬起和落下事件的索引
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * @brief 更新单足的步态时间表（抬起和落下时间的序列）
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * @brief 检查事件索引是否有效
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  /**
   * @brief 根据摆动时间对轨迹进行缩放
   */
  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  const Config config_;
  const size_t numFeet_;

  // 存储每条腿的摆动高度轨迹（样条曲线）和事件时间
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;
};

/**
 * @brief 从配置文件加载摆动轨迹规划器的设置
 */
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config", bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
