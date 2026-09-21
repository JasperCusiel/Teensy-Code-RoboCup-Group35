//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "motion-controller.h"
#include "odometry.h"
#include <math.h>
#include <wiring.h>
#include "math_utils.h"

// This module handles the generation of motor wheel speed commands based on commanded heading and speed.

namespace
{
    bool override_active = false;
    float override_heading = 0.0f;
    // Robot and PID config
    constexpr float kTrackWidthM = 0.28f;
    constexpr float kMaxWheelSpeedMps = 0.50f;
    constexpr float kHeadingKp = 2.5f;
    constexpr float kHeadingKi = 0.00f;
    constexpr float kHeadingKd = 0.00f;

    bool enabled = false; // Tracks if motion controller is enabled.
    drive_output_callback_t output_callback = nullptr; // Callback that sets wheel speed targets in hardware.

    // Target wheel speeds
    float left_target = 0.0f;
    float right_target = 0.0f;

    // Used in heading PID loop
    float heading_integral = 0.0f;
    float heading_previous_error = 0.0f;

    // Used to track dt for PI loops
    uint32_t previous_update_us = 0;
    float take_dt()
    {
        const uint32_t now = micros();
        float dt = static_cast<float>(now - previous_update_us) * 1.0e-6f;
        previous_update_us = now;
        if (dt <= 0.0f || dt > 0.25f) dt = 1.0f / 95.0f;
        return dt;
    }
    float clamp_value(float value, float minimum, float maximum)
    {
        return fminf(maximum, fmaxf(minimum, value));
    }

    void reset_pid()
    {
        heading_integral = 0.0f;
        heading_previous_error = 0.0f;
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
    // Command zero wheel speed.
    left_target = 0.0f;
    right_target = 0.0f;
    reset_pid();
    motion_controller_apply_motor_output(0.0f, 0.0f);
}

void motion_controller_update(const pose_t* pose, const velocity_command_t* command)
{
    // Updates the commanded wheel speeds based on the commanded linear velocity and heading.
    if (override_active) return;   // pickup owns the wheels

    const float dt = take_dt();

    if (!enabled || pose == nullptr || command == nullptr || command->stop)
    {
        motion_controller_stop();
        return;
    }

    // Update heading PID loop. Motor speed control is handled in drivetrain.cpp
    // by DCMotorTacho using each wheel encoder.
    const float turn_rate = heading_pid_update(command->heading, pose->theta, dt) + command->turn_rate;

    // Convert heading and speed to target wheel speeds.
    motion_controller_set_wheel_targets(command->linear_speed - 0.5f * kTrackWidthM * turn_rate,
                                        command->linear_speed + 0.5f * kTrackWidthM * turn_rate);

    // Output to motor controller.
    motion_controller_apply_motor_output(left_target, right_target);
}

void motion_controller_set_enabled(bool new_enabled)
{
    // Enable / disable motion controller
    enabled = new_enabled;
    if (!enabled) motion_controller_stop();
}

void motion_controller_set_output_callback(drive_output_callback_t callback)
{
    // Sets callback that applies wheel speed targets to the motor drives.
    output_callback = callback;
}

float heading_pid_update(float target_heading, float current_heading, float dt)
{
    // PID loop to follow commanded heading.

    const float error = wrap_angle_rad(target_heading - current_heading);

    heading_integral = clamp_value(heading_integral + error * dt, -1.0f, 1.0f);
    const float derivative = (error - heading_previous_error) / dt;

    heading_previous_error = error;

    return kHeadingKp * error + kHeadingKi * heading_integral + kHeadingKd * derivative;
}

void motion_controller_set_wheel_targets(float left_speed, float right_speed)
{
    // Clamp values between max and min wheel speeds.
    left_target = clamp_value(left_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
    right_target = clamp_value(right_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
}

void motion_controller_apply_motor_output(float left, float right)
{
    // Call the motor control callback to set the actual wheel speed targets.
    if (output_callback != nullptr) output_callback(left, right);
}


void motion_controller_get_outputs(float* left, float* right)
{
    // Returns the commanded wheel speed targets.
    if (left != nullptr) *left = left_target;
    if (right != nullptr) *right = right_target;
}

void motion_controller_set_override(bool active)
{
    if (active == override_active) return;
    override_active = active;
    reset_pid();
    motion_controller_stop();          // always start and end from a clean stop

    if (active)
    {
        float x, y, theta;
        get_ekf_pose(&x, &y, &theta);
        override_heading = theta;      // hold the heading we were facing
    }
}

void motion_controller_override_drive(float speed_mps, float turn_rate_rad_s)
{
    if (!override_active || !enabled) return;

    float x, y, theta;
    get_ekf_pose(&x, &y, &theta);
    const float dt = take_dt();

    // Real stop: zero speed and zero turn means hold still
    if (speed_mps == 0.0f && turn_rate_rad_s == 0.0f)
    {
        override_heading = theta;   // re-capture so the next drive holds this heading
        reset_pid();
        motion_controller_set_wheel_targets(0.0f, 0.0f);
        motion_controller_apply_motor_output(0.0f, 0.0f);
        return;
    }

    float turn;
    if (turn_rate_rad_s == 0.0f)
    {
        // Driving straight: hold heading, but ignore tiny errors (~3 degrees)
        const float error = wrap_angle_rad(override_heading - theta);
        if (fabsf(error) < 0.05f)
        {
            turn = 0.0f;
        }
        else
        {
            turn = heading_pid_update(override_heading, theta, dt);
        }
    }
    else
    {
        // Turning: command the rate directly and re-capture the heading
        turn = turn_rate_rad_s;
        override_heading = theta;
        reset_pid();
    }

    motion_controller_set_wheel_targets(speed_mps - 0.5f * kTrackWidthM * turn,
                                        speed_mps + 0.5f * kTrackWidthM * turn);
    motion_controller_apply_motor_output(left_target, right_target);
}