//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "motion-controller.h"
#include "odometry.h"
#include <math.h>
#include <wiring.h>

// This module handles the generation of motor wheel speed commands based on commanded heading and speed.

namespace
{
    // Robot and PID config
    constexpr float kTrackWidthM = 0.28f;
    constexpr float kMaxWheelSpeedMps = 0.50f;
    constexpr float kHeadingKp = 0.5f;
    constexpr float kHeadingKi = 0.00f;
    constexpr float kHeadingKd = 0.00f;
    constexpr float kSpeedKp = 1.0f;
    constexpr float kSpeedKi = 0.0f;
    constexpr float kSpeedKd = 0.00f;
    constexpr float kMaxMotorEffort = 1.0f; // Max motor effort to be commanded, normalised between [-1, 1].

    bool enabled = false; // Tracks if motion controller is enabled.
    drive_output_callback_t output_callback = nullptr; // Callback that sets the actual motor drive output in hardware.

    // Target wheel speeds
    float left_target = 0.0f;
    float right_target = 0.0f;

    // Target motor output effort (capped by kMaxMotorEffort)
    float left_output = 0.0f;
    float right_output = 0.0f;

    // Linear velocity from odom.
    float measured_forward_speed = 0.0f;

    // Used in heading PID loop
    float heading_integral = 0.0f;
    float heading_previous_error = 0.0f;
    // Used in speed PID loop
    float speed_integral = 0.0f;
    float speed_previous_error = 0.0f;

    // Used to track dt for PI loops
    uint32_t previous_update_us = 0;

    float clamp_value(float value, float minimum, float maximum)
    {
        return fminf(maximum, fmaxf(minimum, value));
    }

    float wrap_angle(float angle)
    {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    void reset_pid()
    {
        heading_integral = 0.0f;
        heading_previous_error = 0.0f;
        speed_integral = 0.0f;
        speed_previous_error = 0.0f;
    }
} // namespace

void motion_controller_init()
{
    // Initialize motion controller.
    enabled = true;
    previous_update_us = micros();
    reset_pid();
    motion_controller_stop();
}

void motion_controller_stop()
{
    // Command zero motor effort (stopped)
    left_target = 0.0f;
    right_target = 0.0f;
    left_output = 0.0f;
    right_output = 0.0f;
    reset_pid();
    motion_controller_apply_motor_output(0.0f, 0.0f);
}

void motion_controller_update(const pose_t* pose, const velocity_command_t* command)
{
    // Updates the commanded motor effort based on the commanded linear velocity and heading.
    // Calculate dt for PID loops
    const uint32_t now = micros();
    float dt = static_cast<float>(now - previous_update_us) * 1.0e-6f;
    previous_update_us = now;
    if (dt <= 0.0f || dt > 0.25f) dt = 1.0f / 95.0f;

    if (!enabled || pose == nullptr || command == nullptr || command->stop)
    {
        motion_controller_stop();
        return;
    }

    measured_forward_speed = get_vy();

    // Update heading and speed PID loops
    const float turn_rate = heading_pid_update(command->heading, pose->theta, dt) + command->turn_rate;
    const float drive_effort = speed_pid_update(command->linear_speed, measured_forward_speed, dt);

    // Convert heading and speed to target wheel speeds.
    motion_controller_set_wheel_targets(command->linear_speed - 0.5f * kTrackWidthM * turn_rate,
                                        command->linear_speed + 0.5f * kTrackWidthM * turn_rate);

    // Normalize wheel speeds to motor effort.
    const float turn_effort = 0.5f * kTrackWidthM * turn_rate / kMaxWheelSpeedMps;

    left_output = clamp_value(drive_effort - turn_effort, -kMaxMotorEffort, kMaxMotorEffort);
    right_output = clamp_value(drive_effort + turn_effort, -kMaxMotorEffort, kMaxMotorEffort);

    // Output to motor controller.
    motion_controller_apply_motor_output(left_output, right_output);
}

void motion_controller_set_enabled(bool new_enabled)
{
    // Enable / disable motion controller
    enabled = new_enabled;
    if (!enabled) motion_controller_stop();
}

void motion_controller_set_output_callback(drive_output_callback_t callback)
{
    // Sets callback that sets the actual speed for the motor drives.
    output_callback = callback;
}

float heading_pid_update(float target_heading, float current_heading, float dt)
{
    // PID loop to follow commanded heading.

    const float error = wrap_angle(target_heading - current_heading);

    heading_integral = clamp_value(heading_integral + error * dt, -1.0f, 1.0f);
    const float derivative = (error - heading_previous_error) / dt;

    heading_previous_error = error;

    return kHeadingKp * error + kHeadingKi * heading_integral + kHeadingKd * derivative;
}

float speed_pid_update(float target_speed, float current_speed, float dt)
{
    // Speed PID loop
    const float error = target_speed - current_speed;

    speed_integral = clamp_value(speed_integral + error * dt, -1.0f, 1.0f);
    const float derivative = (error - speed_previous_error) / dt;

    speed_previous_error = error;

    return clamp_value(kSpeedKp * error + kSpeedKi * speed_integral + kSpeedKd * derivative, -1.0f, 1.0f);
}

void motion_controller_set_wheel_targets(float left_speed, float right_speed)
{
    // Clamp values between max and min wheel speeds.
    left_target = clamp_value(left_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
    right_target = clamp_value(right_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
}

void motion_controller_apply_motor_output(float left, float right)
{
    // Call the motor control callback to set the actual motor speeds.
    if (output_callback != nullptr) output_callback(left, right);
}


void motion_controller_get_outputs(float* left, float* right)
{
    // Returns the commanded motor output output
    if (left != nullptr) *left = left_output;
    if (right != nullptr) *right = right_output;
}

