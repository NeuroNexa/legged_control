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

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>

// The file is located in legged_interface, but the namespace is ocs2::legged_robot for consistency with the ocs2 framework.
namespace ocs2 {
namespace legged_robot {

/**
 * @class SwingTrajectoryPlanner
 * @brief Plans the swing motion for the feet of a legged robot.
 * It generates smooth trajectories for the height (z-component) of the feet during swing phases
 * using cubic splines (SplineCpg). This allows for specifying liftoff/touchdown velocities and swing height.
 */
class SwingTrajectoryPlanner {
 public:
  /**
   * @struct Config
   * @brief Configuration parameters for the swing trajectory planner.
   */
  struct Config {
    scalar_t liftOffVelocity = 0.0;      //!< The desired vertical velocity of the foot at liftoff.
    scalar_t touchDownVelocity = 0.0;    //!< The desired vertical velocity of the foot at touchdown.
    scalar_t swingHeight = 0.1;          //!< The desired maximum height of the foot during swing.
    scalar_t swingTimeScale = 0.15;      //!< Swing phases shorter than this will have their height and velocity scaled down.
  };

  /**
   * @brief Constructor for the SwingTrajectoryPlanner.
   * @param config : The configuration for the planner.
   * @param numFeet : The number of feet of the robot.
   */
  SwingTrajectoryPlanner(Config config, size_t numFeet);

  /**
   * @brief Updates the swing trajectories based on a mode schedule and a constant terrain height.
   * @param modeSchedule : The planned mode schedule defining stance and swing phases.
   * @param terrainHeight : The assumed constant height of the terrain.
   */
  void update(const ModeSchedule& modeSchedule, scalar_t terrainHeight);

  /**
   * @brief Updates the swing trajectories with varying liftoff and touchdown heights.
   * @param modeSchedule : The planned mode schedule.
   * @param liftOffHeightSequence : A sequence of desired liftoff heights for each foot.
   * @param touchDownHeightSequence : A sequence of desired touchdown heights for each foot.
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence);

  /**
   * @brief Updates the swing trajectories with varying liftoff, touchdown, and maximum swing heights.
   * @param modeSchedule : The planned mode schedule.
   * @param liftOffHeightSequence : A sequence of desired liftoff heights for each foot.
   * @param touchDownHeightSequence : A sequence of desired touchdown heights for each foot.
   * @param maxHeightSequence : A sequence of desired maximum swing heights for each foot.
   */
  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence, const feet_array_t<scalar_array_t>& maxHeightSequence);

  /**
   * @brief Gets the desired vertical velocity of a foot at a specific time.
   * @param leg : The index of the leg.
   * @param time : The time to query.
   * @return The desired vertical velocity.
   */
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  /**
   * @brief Gets the desired vertical position (height) of a foot at a specific time.
   * @param leg : The index of the leg.
   * @param time : The time to query.
   * @return The desired vertical position.
   */
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

 private:
  /**
   * @brief Extracts the contact sequence for each leg from a sequence of mode IDs.
   * @param phaseIDsStock : Vector of mode IDs.
   * @return An array of boolean vectors, where each vector represents the contact sequence for a foot.
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * @brief Finds the indices of the take-off and touch-down events for a swing phase.
   * @param index : The current index in the contact sequence.
   * @param contactFlagStock : The contact sequence for a single leg.
   * @return A pair containing the take-off time index and touch-down time index.
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * @brief Determines the start and end event indices for all swing phases of a foot.
   * @param contactFlagStock : The contact sequence for a single leg.
   * @return A pair of vectors, containing the start and final event time indices for each swing phase.
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * @brief Checks if the calculated event time indices for a swing phase are valid.
   * @param leg : The leg index.
   * @param index : The phase index.
   * @param startIndex : The calculated liftoff event time index.
   * @param finalIndex : The calculated touchdown event time index.
   * @param phaseIDsStock : The sequence of mode IDs.
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  /**
   * @brief Calculates a scaling factor for swing motion based on the duration of the swing phase.
   * Short swing phases are scaled down to avoid aggressive motions.
   * @param startTime : The start time of the swing phase.
   * @param finalTime : The end time of the swing phase.
   * @param swingTimeScale : The time scale parameter from the config.
   * @return A scaling factor between 0 and 1.
   */
  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  const Config config_;
  const size_t numFeet_;

  // Storage for the generated spline trajectories.
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  // Storage for the event times associated with the trajectories.
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;
};

/**
 * @brief Loads swing trajectory settings from a configuration file.
 * @param fileName : Path to the configuration file.
 * @param fieldName : The name of the field in the config file.
 * @param verbose : Whether to print loaded values.
 * @return An instance of SwingTrajectoryPlanner::Config.
 */
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config", bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
