//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_NAVIGATION_TYPES_H
#define ROBOCUP_NAVIGATION_TYPES_H

#include "astar.h"
#include <stdbool.h>

typedef enum {
  NAV_GOAL_NONE,
  NAV_GOAL_FRONTIER,
  NAV_GOAL_BASE
} navigation_goal_type_t;

typedef struct {
  navigation_goal_type_t type;
  grid_point_t cell;
} navigation_goal_t;

typedef struct {
  float heading;       // Desired world-frame direction, radians
  float linear_speed;  // Desired forward speed, m/s
  float turn_rate;     // Desired angular speed feed-forward, rad/s
  bool stop;
} velocity_command_t;

typedef enum {
  NAV_STATUS_IDLE,
  NAV_STATUS_PLANNING,
  NAV_STATUS_FOLLOWING_PATH,
  NAV_STATUS_GOAL_REACHED,
  NAV_STATUS_EXPLORATION_COMPLETE,
  NAV_STATUS_PATH_FAILED
} navigation_status_t;

#endif // ROBOCUP_NAVIGATION_TYPES_H
