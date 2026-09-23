//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_MOTION_CONTROLLER_H
#define ROBOCUP_MOTION_CONTROLLER_H

#include "navigation-types.h"
#include "odometry.h"

// Wheel speed targets are in metres per second.
typedef void (*drive_output_callback_t)(float left_mps,
                                        float right_mps);

void motion_controller_init();
void motion_controller_stop();

void motion_controller_update(
    const pose_t* pose,
    const velocity_command_t* command);

void motion_controller_set_enabled(bool enabled);

float heading_pid_update(float target_heading, float current_heading, float dt);

void motion_controller_set_wheel_targets(
    float left_speed,
    float right_speed);


void motion_controller_get_outputs(float* left_mps, float* right_mps);

void motion_controller_set_override(bool active);
void motion_controller_override_drive(float speed_mps, float turn_rate_rad_s);

#endif // ROBOCUP_MOTION_CONTROLLER_H
