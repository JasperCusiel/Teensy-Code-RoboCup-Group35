//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_PURE_PURSUIT_H
#define ROBOCUP_PURE_PURSUIT_H

#include "navigation-types.h"
#include "odometry.h"

// Generate new velocity command based on robot position along path.
velocity_command_t pure_pursuit_update(const path_t* path, const pose_t* robot_pose);

// Mark path complete when we are within a tolerance of the final path point.
bool pure_pursuit_path_complete(const path_t* path, const pose_t* robot_pose);

#endif // ROBOCUP_PURE_PURSUIT_H
