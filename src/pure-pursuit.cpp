//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "pure-pursuit.h"

#include "occupancy-grid.h"

#include <math.h>

namespace {

constexpr float kLookaheadDistanceM = 0.35f;
constexpr float kGoalToleranceM = 0.15f;
constexpr float kNominalSpeedMps = 0.30f;
constexpr float kMinimumTrackingSpeedMps = 0.08f;

float wrap_angle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

void cell_to_world(grid_point_t cell, float *x, float *y) {
  *x = MAP_WORLD_MIN_X + (static_cast<float>(cell.x) + 0.5f) * MAP_M_PER_CELL;
  *y = MAP_WORLD_MIN_Y + (static_cast<float>(cell.y) + 0.5f) * MAP_M_PER_CELL;
}

float squared_distance_to_cell(const pose_t *pose, grid_point_t cell) {
  float x = 0.0f;
  float y = 0.0f;
  cell_to_world(cell, &x, &y);
  const float dx = x - pose->x;
  const float dy = y - pose->y;
  return dx * dx + dy * dy;
}

int find_lookahead_index(const path_t *path, const pose_t *pose) {
  int nearest = 0;
  float nearest_distance = squared_distance_to_cell(pose, path->points[0]);
  for (int i = 1; i < path->length; ++i) {
    const float distance = squared_distance_to_cell(pose, path->points[i]);
    if (distance <= nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }
  float accumulated = 0.0f;
  int lookahead = nearest;
  for (int i = nearest + 1; i < path->length; ++i) {
    const int dx = path->points[i].x - path->points[i - 1].x;
    const int dy = path->points[i].y - path->points[i - 1].y;
    accumulated += sqrtf(static_cast<float>(dx * dx + dy * dy)) * MAP_M_PER_CELL;
    lookahead = i;
    if (accumulated >= kLookaheadDistanceM) break;
  }
  return lookahead;
}

float heading_to_cell(const pose_t *pose, grid_point_t cell) {
  float x = 0.0f;
  float y = 0.0f;
  cell_to_world(cell, &x, &y);
  // This project defines heading zero along world +Y and positive CCW.
  return atan2f(-(x - pose->x), y - pose->y);
}

} // namespace

void pure_pursuit_init() { pure_pursuit_reset(); }

velocity_command_t pure_pursuit_update(const path_t *path,
                                       const pose_t *robot_pose) {
  velocity_command_t command = {robot_pose != nullptr ? robot_pose->theta : 0.0f,
                                0.0f, 0.0f, true};
  if (path == nullptr || robot_pose == nullptr || path->length == 0 ||
      pure_pursuit_path_complete(path, robot_pose)) {
    return command;
  }

  const int lookahead_index = find_lookahead_index(path, robot_pose);
  command.heading = heading_to_cell(robot_pose, path->points[lookahead_index]);
  const float heading_error = wrap_angle(command.heading - robot_pose->theta);
  const float alignment = fmaxf(0.0f, cosf(heading_error));
  command.linear_speed = alignment > 0.0f
                             ? fmaxf(kMinimumTrackingSpeedMps,
                                     kNominalSpeedMps * alignment)
                             : 0.0f;
  command.turn_rate = 0.0f;
  command.stop = false;
  return command;
}

bool pure_pursuit_path_complete(const path_t *path,
                                const pose_t *robot_pose) {
  if (path == nullptr || robot_pose == nullptr || path->length == 0) {
    return true;
  }
  return squared_distance_to_cell(robot_pose, path->points[path->length - 1]) <=
         kGoalToleranceM * kGoalToleranceM;
}

void pure_pursuit_reset() {}
