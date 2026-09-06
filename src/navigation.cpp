//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "navigation.h"

#include "frontier-detection.h"
#include "mission.h"
#include "odometry.h"

namespace {

constexpr float kGoalToleranceM = 0.5f;
constexpr uint8_t kNoFrontierConfirmationCycles = 20;

navigation_goal_t active_goal = {NAV_GOAL_NONE, {0, 0}};
path_t active_path = {};
navigation_status_t status = NAV_STATUS_IDLE;
grid_point_t base_cell = {0, 0};
bool base_is_set = false;
bool replan_requested = false;
uint8_t no_frontier_cycles = 0;

bool cells_equal(grid_point_t a, grid_point_t b) {
  return a.x == b.x && a.y == b.y;
}

void get_pose(pose_t *pose) {
  get_ekf_pose(&pose->x, &pose->y, &pose->theta);
}

float cell_world_x(int x) {
  return MAP_WORLD_MIN_X + (static_cast<float>(x) + 0.5f) * MAP_M_PER_CELL;
}

float cell_world_y(int y) {
  return MAP_WORLD_MIN_Y + (static_cast<float>(y) + 0.5f) * MAP_M_PER_CELL;
}

bool pose_at_goal(const pose_t &pose) {
  if (active_goal.type == NAV_GOAL_NONE) {
    return false;
  }
  const float dx = cell_world_x(active_goal.cell.x) - pose.x;
  const float dy = cell_world_y(active_goal.cell.y) - pose.y;
  return dx * dx + dy * dy <= kGoalToleranceM * kGoalToleranceM;
}

bool choose_frontier(const grid_point_t &robot_cell) {
  frontier_goal_t frontier = {};
  if (!frontier_find_largest_goal(robot_cell.x, robot_cell.y, &frontier)) {
    active_goal = {NAV_GOAL_NONE, {0, 0}};
    active_path.length = 0;
    if (no_frontier_cycles < kNoFrontierConfirmationCycles) {
      ++no_frontier_cycles;
    }
    status = no_frontier_cycles >= kNoFrontierConfirmationCycles
                 ? NAV_STATUS_EXPLORATION_COMPLETE
                 : NAV_STATUS_IDLE;
    return false;
  }

  no_frontier_cycles = 0;

  navigation_set_goal(
      {NAV_GOAL_FRONTIER, {frontier.x, frontier.y}});
  return true;
}

bool plan_from(const grid_point_t &start) {
  status = NAV_STATUS_PLANNING;
  active_path.length = 0;
  replan_requested = false;
  if (!astar_find_path(start.x, start.y, active_goal.cell.x,
                       active_goal.cell.y, &active_path)) {
    status = NAV_STATUS_PATH_FAILED;
    return false;
  }
  status = NAV_STATUS_FOLLOWING_PATH;
  return true;
}

} // namespace

void navigation_init() {
  active_goal = {NAV_GOAL_NONE, {0, 0}};
  active_path.length = 0;
  status = NAV_STATUS_IDLE;
  replan_requested = false;
  no_frontier_cycles = 0;

  pose_t pose = {};
  get_pose(&pose);
  base_is_set = world_to_map(pose.x, pose.y, &base_cell.x, &base_cell.y);
}

void navigation_task() {
  if (mission_should_stop()) {
    if (active_goal.type != NAV_GOAL_NONE) {
      navigation_clear_goal();
    }
    return;
  }

  pose_t pose = {};
  get_pose(&pose);
  grid_point_t robot_cell = {};
  if (!world_to_map(pose.x, pose.y, &robot_cell.x, &robot_cell.y)) {
    active_path.length = 0;
    status = NAV_STATUS_PATH_FAILED;
    return;
  }

  if (pose_at_goal(pose)) {
    active_path.length = 0;
    status = NAV_STATUS_GOAL_REACHED;
    if (mission_should_explore()) {
      active_goal = {NAV_GOAL_NONE, {0, 0}};
    } else {
      return;
    }
  }

  if (mission_should_explore() && active_goal.type == NAV_GOAL_NONE) {
    if (!choose_frontier(robot_cell)) {
      return;
    }
  } else if (mission_should_return_home()) {
    if (!base_is_set) {
      status = NAV_STATUS_PATH_FAILED;
      return;
    }
    if (active_goal.type != NAV_GOAL_BASE ||
        !cells_equal(active_goal.cell, base_cell)) {
      navigation_set_goal({NAV_GOAL_BASE, base_cell});
    }
  }

  if (active_goal.type != NAV_GOAL_NONE &&
      (active_path.length == 0 || replan_requested)) {
    plan_from(robot_cell);
  }
}

void navigation_set_goal(navigation_goal_t goal) {
  if (goal.type == NAV_GOAL_NONE) {
    navigation_clear_goal();
    return;
  }
  if (active_goal.type != goal.type ||
      !cells_equal(active_goal.cell, goal.cell)) {
    active_goal = goal;
    active_path.length = 0;
    replan_requested = true;
  }
}

void navigation_clear_goal() {
  active_goal = {NAV_GOAL_NONE, {0, 0}};
  active_path.length = 0;
  replan_requested = false;
  no_frontier_cycles = 0;
  status = NAV_STATUS_IDLE;
}

bool navigation_has_goal() { return active_goal.type != NAV_GOAL_NONE; }

navigation_goal_t navigation_get_goal() { return active_goal; }

bool navigation_has_path() { return active_path.length > 0; }

const path_t *navigation_get_path() {
  return navigation_has_path() ? &active_path : nullptr;
}

bool navigation_goal_reached() { return status == NAV_STATUS_GOAL_REACHED; }

bool navigation_exploration_complete() {
  return status == NAV_STATUS_EXPLORATION_COMPLETE;
}

bool navigation_path_failed() { return status == NAV_STATUS_PATH_FAILED; }

navigation_status_t navigation_get_status() { return status; }

void navigation_request_replan() {
  if (active_goal.type != NAV_GOAL_NONE) {
    replan_requested = true;
  }
}

void navigation_set_base(float world_x, float world_y) {
  base_is_set = world_to_map(world_x, world_y, &base_cell.x, &base_cell.y);
}
