//
// Created by Jasper Cusiel on 16/09/2026.
//

#ifndef ROBOCUP_COVERAGE_PLANNER_H
#define ROBOCUP_COVERAGE_PLANNER_H

#include "astar.h"
#include <stdint.h>

void coverage_planner_init();
void coverage_planner_reset();
void coverage_planner_start();
bool coverage_planner_started();

// Returns the current lawnmower survey goal, skipping cells that are no longer safe.
bool coverage_planner_get_goal(grid_point_t* goal);

// Advance past the current survey goal after reaching it or deciding it cannot be used.
void coverage_planner_advance_goal();
void coverage_planner_reject_goal();

bool coverage_planner_complete();
uint16_t coverage_planner_goal_count();
uint16_t coverage_planner_goal_index();

#endif // ROBOCUP_COVERAGE_PLANNER_H
