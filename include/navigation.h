//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_NAVIGATION_H
#define ROBOCUP_NAVIGATION_H

#include "navigation-types.h"

// Initialize nav module.
void navigation_init();

// Main loop, call periodically to convert mission intent into nav goal.
void navigation_task();

// Set new goal
void navigation_set_goal(navigation_goal_t goal);

// Reset navigation to an idle state with no target or path.
void navigation_clear_goal();

// Re calculate path to goal
void navigation_request_replan();

// Getter functions.
bool navigation_has_goal();
navigation_goal_t navigation_get_goal();
bool navigation_has_path();

// Returns nullptr when no valid path available.
const path_t* navigation_get_path();
bool navigation_goal_reached();
bool navigation_exploration_complete();
bool navigation_path_failed();
navigation_status_t navigation_get_status();

// Sets base world location.
void navigation_set_base(float world_x, float world_y);

#endif // ROBOCUP_NAVIGATION_H
