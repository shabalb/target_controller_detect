#pragma once

#include <algorithm>
#include <cmath>

#include "target_controller_detect/msg/target_state.hpp"

namespace target_controller_detect {

enum class FollowMode { SEARCH, ALIGN, FOLLOW, STOP, LOST };

struct ControllerConfig {
  float kd = 0.8f;
  float ka = 1.5f;
  float max_linear = 0.3f;
  float max_angular = 0.1f;
  uint16_t max_linear_track = 2000;
  uint16_t min_linear_track = 0;
  float desired_distance = 0.7f;
  float dist_deadband = 0.25f;
  float angle_align_deadband = 0.05f;
  float angle_follow_deadband = 0.02f;
  float lost_search_angular = 0.07f;
  uint16_t lost_left = 500;
};

struct MotionCommand {
  float linear = 0.0f;
  float angular = 0.0f;
};

class MotionCommandTrack {
  public:
  uint16_t right = 0;// от 0 до 2000
  uint16_t left = 0;// от 0 до 2000
};

inline FollowMode decideMode(
  const target_controller_detect::msg::TargetState &target,
  const ControllerConfig &config = {}) {
  if (!target.valid) {
    return FollowMode::LOST;
  }

  if (std::fabs(target.angle) > config.angle_align_deadband) {
    return FollowMode::ALIGN;
  }

  const float dist_error = target.distance - config.desired_distance;
  if (
    std::fabs(dist_error) < config.dist_deadband &&
    std::fabs(target.angle) < config.angle_follow_deadband)
  {
    return FollowMode::STOP;
  }

  return FollowMode::FOLLOW;
}

inline MotionCommandTrack computeCommandTrack(
  const target_controller_detect::msg::TargetState &target,
  const FollowMode mode,
  const ControllerConfig &config = {}) 
  {
  MotionCommandTrack cmd;
  const float dist_error = target.distance - config.desired_distance;
  const float angle_error = target.angle;
  float angle = 0.;
  switch (mode) {
    case FollowMode::ALIGN:
      angle = -std::clamp(
        config.ka * angle_error, -config.max_angular, config.max_angular);
      if (angle < 0){
        cmd.right = 1200;
      }else {
        cmd.left = 1200;
      }
      break;
    case FollowMode::FOLLOW:
      if (std::fabs(dist_error) >= config.dist_deadband) {
        cmd.left = cmd.right = std::clamp(
          (uint16_t)(config.kd * dist_error*1000), config.min_linear_track, config.max_linear_track);
      }
      if (std::fabs(angle_error) >= config.angle_follow_deadband) {
        angle = -std::clamp(
          config.ka * angle_error, -config.max_angular, config.max_angular);
        if (angle < 0){
          cmd.right = 1200;
        }else {
          cmd.left = 1200;
        }
      }
      break;
    case FollowMode::STOP:
      break;
    case FollowMode::SEARCH:
      break;
    case FollowMode::LOST:
      cmd.left = 800;
      cmd.right = 1200;
      break;
  }

  return cmd;
}


inline MotionCommand computeCommand(
  const target_controller_detect::msg::TargetState &target,
  const FollowMode mode,
  const ControllerConfig &config = {}) 
  {
  MotionCommand cmd;
  const float dist_error = target.distance - config.desired_distance;
  const float angle_error = target.angle;

  switch (mode) {
    case FollowMode::ALIGN:
      cmd.angular = -std::clamp(
        config.ka * angle_error, -config.max_angular, config.max_angular);
      break;
    case FollowMode::FOLLOW:
      if (std::fabs(dist_error) >= config.dist_deadband) {
        cmd.linear = std::clamp(
          config.kd * dist_error, -config.max_linear, config.max_linear);
      }
      if (std::fabs(angle_error) >= config.angle_follow_deadband) {
        cmd.angular = -std::clamp(
          config.ka * angle_error, -config.max_angular, config.max_angular);
      }
      break;
    case FollowMode::STOP:
      break;
    case FollowMode::SEARCH:
    case FollowMode::LOST:
      cmd.angular = config.lost_left;
      break;
  }

  return cmd;
}

}  // namespace target_controller_detect
