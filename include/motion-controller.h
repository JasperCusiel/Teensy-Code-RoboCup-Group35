//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_MOTION_CONTROLLER_H
#define ROBOCUP_MOTION_CONTROLLER_H

#include "navigation-types.h"
#include "odometry.h"

// Motor outputs are normalized efforts in the range [-1, 1].
typedef void (*drive_output_callback_t)(float left_output,
                                        float right_output);

void motion_controller_init();
void motion_controller_stop();

void motion_controller_update(
    const pose_t* pose,
    const velocity_command_t* command);

void motion_controller_set_enabled(bool enabled);
void motion_controller_set_output_callback(drive_output_callback_t callback);

float heading_pid_update(float target_heading, float current_heading, float dt);
float speed_pid_update(float target_speed, float current_speed, float dt);

void motion_controller_set_wheel_targets(
    float left_speed,
    float right_speed);

void motion_controller_apply_motor_output(
    float left_output,
    float right_output);

void motion_controller_get_outputs(float* left_output, float* right_output);

#endif // ROBOCUP_MOTION_CONTROLLER_H
