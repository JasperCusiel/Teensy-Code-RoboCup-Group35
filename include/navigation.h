//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_NAVIGATION_H
#define ROBOCUP_NAVIGATION_H

#include "mapping.h"
#include "navigation-types.h"

void navigation_init();
void navigation_task();

void navigation_set_goal(navigation_goal_t goal);
void navigation_clear_goal();

bool navigation_has_goal();
navigation_goal_t navigation_get_goal();

bool navigation_has_path();
const path_t *navigation_get_path();

bool navigation_goal_reached();
bool navigation_exploration_complete();
bool navigation_path_failed();
navigation_status_t navigation_get_status();
void navigation_request_replan();

void navigation_set_base(float world_x, float world_y);

#endif // ROBOCUP_NAVIGATION_H
