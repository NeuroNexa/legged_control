/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include <ocs2_legged_robot/common/Types.hh>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>

// 该文件位于 legged_interface 中，但为了与 ocs2 框架保持一致，命名空间为 ocs2::legged_robot。
namespace ocs2 {
namespace legged_robot {

/**
 * @class SwingTrajectoryPlanner
 * @brief 为足式机器人的脚规划摆动轨迹。
 * 它使用三次样条（SplineCpg）为摆动阶段的脚的高度（z分量）生成平滑轨迹。
 * 这允许指定抬脚/触地速度和摆动高度。
 */
class SwingTrajectoryPlanner {
 public:
  /**
   * @struct Config
   * @brief 摆动轨迹规划器的配置参数。
   */
  struct Config {
    scalar_t liftOffVelocity = 0.0;      //!< 脚抬起时的期望垂直速度。
    scalar_t touchDownVelocity = 0.0;    //!< 脚触地时的期望垂直速度。
    scalar_t swingHeight = 0.1;          //!< 摆动期间脚的期望最大高度。
    scalar_t swingTimeScale = 0.15;      //!< 比此时间短的摆动阶段的高度和速度将被按比例缩小。
  };

  /**
   * @brief SwingTrajectoryPlanner 的构造函数。
   * @param config 规划器的配置。
   * @param numFeet 机器人的脚数。
   */
  SwingTrajectoryPlanner(Config config, size_t numFeet);

  /**
   * @brief 根据模式计划和恒定的地形高度更新摆动轨迹。
   * @param modeSchedule 规划的模式计划，定义了支撑和摆动阶段。
   * @param terrainHeight 假定的恒定地形高度。
   */
  void update(const ModeSchedule& modeSchedule, scalar_t terrainHeight);

  /**
   * @brief 使用变化的抬脚和触地高度更新摆动轨迹。
   * @param modeSchedule 规划的模式计划。
   * @param liftOffHeightSequence 每只脚的期望抬脚高度序列。
   * @param touchDownHeightSequence 每只脚的期望触地高度序列。
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence);

  /**
   * @brief 使用变化的抬脚、触地和最大摆动高度更新摆动轨迹。
   * @param modeSchedule 规划的模式计划。
   * @param liftOffHeightSequence 每只脚的期望抬脚高度序列。
   * @param touchDownHeightSequence 每只脚的期望触地高度序列。
   * @param maxHeightSequence 每只脚的期望最大摆动高度序列。
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence, const feet_array_t<scalar_array_t>& maxHeightSequence);

  /**
   * @brief 获取特定时间脚的期望垂直速度。
   * @param leg 腿的索引。
   * @param time 要查询的时间。
   * @return 期望的垂直速度。
   */
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  /**
   * @brief 获取特定时间脚的期望垂直位置（高度）。
   * @param leg 腿的索引。
   * @param time 要查询的时间。
   * @return 期望的垂直位置。
   */
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

 private:
  /**
   * @brief 从模式ID序列中提取每条腿的接触序列。
   * @param phaseIDsStock 模式ID的向量。
   * @return 一个布尔向量数组，其中每个向量代表一只脚的接触序列。
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * @brief 查找摆动阶段的起飞和触地事件的索引。
   * @param index 接触序列中的当前索引。
   * @param contactFlagStock 单腿的接触序列。
   * @return 包含起飞时间索引和触地时间索引的 pair。
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * @brief 确定一只脚所有摆动阶段的开始和结束事件索引。
   * @param contactFlagStock 单腿的接触序列。
   * @return 一对向量，包含每个摆动阶段的开始和结束事件时间索引。
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * @brief 检查计算出的摆动阶段事件时间索引是否有效。
   * @param leg 腿索引。
   * @param index 阶段索引。
   * @param startIndex 计算出的抬脚事件时间索引。
   * @param finalIndex 计算出的触地事件时间索引。
   * @param phaseIDsStock 模式ID序列。
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  /**
   * @brief 根据摆动阶段的持续时间计算摆动运动的缩放因子。
   * 较短的摆动阶段会被缩小以避免剧烈运动。
   * @param startTime 摆动阶段的开始时间。
   * @param finalTime 摆动阶段的结束时间。
   * @param swingTimeScale 配置中的时间尺度参数。
   * @return 介于0和1之间的缩放因子。
   */
  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  const Config config_;
  const size_t numFeet_;

  // 用于存储生成的样条轨迹。
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  // 用于存储与轨迹相关的事件时间。
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;
};

/**
 * @brief 从配置文件加载摆动轨迹设置。
 * @param fileName 配置文件的路径。
 * @param fieldName 配置文件中的字段名称。
 * @param verbose 是否打印加载的值。
 * @return SwingTrajectoryPlanner::Config 的一个实例。
 */
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config", bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
