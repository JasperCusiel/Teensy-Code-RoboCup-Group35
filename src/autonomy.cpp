//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "autonomy.h"

#include "mission.h"
#include "motion-controller.h"
#include "navigation.h"
#include "pure-pursuit.h"
#include "vfh.h"

#include "odometry.h"

#include <math.h>

namespace {

constexpr float kRecoveryHeadingOffsetRad = 0.4f;
constexpr float kSteeringDirectionDeadbandRad = 0.05f;

pose_t current_pose = {};
velocity_command_t safe_command = {0.0f, 0.0f, 0.0f, true}; // Motor command that has been checked with the obstacle avoidance
// Positive is CCW. With no steering history, the first recovery turns CW.
float last_steering_direction = 1.0f;

float wrap_angle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

velocity_command_t avoid_obstacles(const velocity_command_t &target, const pose_t &pose) {

  if (target.stop) {
    return target;
  }

  const float target_relative = wrap_angle(target.heading - pose.theta);
  set_target_angle(target_relative);

  // Run obstacle avoidance every cycle so it can't be bypassed by pure pursuit path follower
  compute_vfh();
  const float steering_relative = get_steering_angle();

  if (!isfinite(steering_relative)) {

    const float recovery_direction = -last_steering_direction;
    return {wrap_angle(pose.theta +
                       recovery_direction * kRecoveryHeadingOffsetRad),
            0.0f, 0.0f, false};
  }

  if (fabsf(steering_relative) > kSteeringDirectionDeadbandRad) {
    last_steering_direction = steering_relative > 0.0f ? 1.0f : -1.0f;
  }

  velocity_command_t safe = target;
  safe.heading = wrap_angle(pose.theta + steering_relative);
  const float deflection = fabsf(wrap_angle(steering_relative - target_relative));
  safe.linear_speed *= fmaxf(0.0f, cosf(deflection));
  safe.turn_rate = 0.0f;
  // A zero forward speed is still a valid rotate-in-place command.
  safe.stop = false;
  return safe;
}

} // namespace

void autonomy_init() {
  get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
  mission_init();
  navigation_init();
  pure_pursuit_init();
  motion_controller_init();
  last_steering_direction = 1.0f;
  safe_command = {current_pose.theta, 0.0f, 0.0f, true};
}

void autonomy_task() {
  get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
  mission_task();
  navigation_task();

  const path_t *path = navigation_get_path();
  const velocity_command_t target = pure_pursuit_update(path, &current_pose);
  safe_command = avoid_obstacles(target, current_pose);

  if (mission_should_stop()) {
    safe_command = {current_pose.theta, 0.0f, 0.0f, true};
  }
}

void autonomy_motion_task() {
  get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
  motion_controller_update(&current_pose, &safe_command);
}
