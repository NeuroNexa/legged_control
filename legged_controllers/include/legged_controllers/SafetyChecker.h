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
 * @brief A class to check if the robot's state is within safe operating limits.
 *
 * This class is responsible for performing safety checks on the robot's current and planned states.
 * If a safety check fails, it can trigger a controller shutdown to prevent damage to the robot.
 * Currently, it only implements a basic check for the robot's orientation (pitch angle).
 * This could be extended to include other checks, such as joint position/velocity limits,
 * torque limits, or collision detection.
 */
class SafetyChecker {
 public:
  /**
   * @brief Constructor for the SafetyChecker.
   * @param info : The centroidal model information.
   */
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  /**
   * @brief The main safety check function.
   * @param observation : The current system observation (state, input, etc.).
   * @param optimized_state : The planned optimal state from the MPC.
   * @param optimized_input : The planned optimal input from the MPC.
   * @return True if all safety checks pass, false otherwise.
   */
  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    // Currently, only the orientation check is performed on the current observation.
    return checkOrientation(observation);
  }

 protected:
  /**
   * @brief Checks if the robot's orientation is within safe limits.
   * @param observation : The current system observation.
   * @return True if the orientation is safe, false otherwise.
   */
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    // Check if the pitch angle (approximated by the 6th element of the base pose, likely Euler ZYX) is within [-pi/2, pi/2].
    if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed! Pitch angle is too high." << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace legged
