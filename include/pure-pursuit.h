//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_PURE_PURSUIT_H
#define ROBOCUP_PURE_PURSUIT_H

#include "mapping.h"
#include "navigation-types.h"

void pure_pursuit_init();

velocity_command_t pure_pursuit_update(
    const path_t *path,
    const pose_t *robot_pose);

bool pure_pursuit_path_complete(
    const path_t *path,
    const pose_t *robot_pose);

void pure_pursuit_reset();

#endif // ROBOCUP_PURE_PURSUIT_H
