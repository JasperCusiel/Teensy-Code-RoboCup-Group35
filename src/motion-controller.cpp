//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "motion-controller.h"
#include "odometry.h"
#include <math.h>
#include <wiring.h>
#include "math_utils.h"
#include "drivetrain.h"

// This module handles the generation of motor wheel speed commands based on commanded heading and speed.

namespace
{
    bool override_active = false;
    float override_heading = 0.0f;
    // Robot and PID config
    constexpr float kTrackWidthM = 0.28f;
    constexpr float kMaxWheelSpeedMps = 0.50f;
    constexpr float kHeadingKp = 4.00f;
    constexpr float kHeadingKi = 0.00f;
    constexpr float kHeadingKd = 0.00f;
    constexpr float kMaxHeadingCorrectionRadPerSec = 2.0f;
    constexpr float kMaxTurnRateRadPerSec = 4.0f;
    constexpr float kMinWheelSpeedDuringHeadingCorrectionMps = 0.03f;
    constexpr float kTurnInPlaceLinearDeadbandMps = 0.01f;
    constexpr float kTurnRateDeadbandRadPerSec = 0.01f;

    bool enabled = false; // Tracks if motion controller is enabled.

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

    float wrap_angle(float angle)
    {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    float max_heading_correction_for_speed(float linear_speed)
    {
        return kMaxHeadingCorrectionRadPerSec;
        const float usable_speed =
            fmaxf(0.0f, fabsf(linear_speed) - kMinWheelSpeedDuringHeadingCorrectionMps);
        return fminf(kMaxHeadingCorrectionRadPerSec,
                     2.0f * usable_speed / kTrackWidthM);
    }

    bool is_direct_turn_rate_command(const velocity_command_t* command)
    {
        return fabsf(command->linear_speed) <= kTurnInPlaceLinearDeadbandMps &&
            fabsf(command->turn_rate) > kTurnRateDeadbandRadPerSec;
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
    set_open_loop_wheel_speed_targets(0.0f, 0.0f);
}

void motion_controller_update(const pose_t* pose, const velocity_command_t* command)
{
    // Updates the commanded wheel speeds based on the commanded linear velocity and heading.
    if (override_active) return; // pickup owns the wheels

    const float dt = take_dt();

    if (!enabled || pose == nullptr || command == nullptr || command->stop)
    {
        motion_controller_stop();
        return;
    }

    float turn_rate = command->turn_rate;
    if (is_direct_turn_rate_command(command))
    {
        reset_pid();
    }
    else
    {
        const float heading_correction_limit =
            max_heading_correction_for_speed(command->linear_speed);
        const float heading_correction = clamp_value(
            heading_pid_update(command->heading, pose->theta, dt),
            -heading_correction_limit,
            heading_correction_limit);
        turn_rate += heading_correction;
    }

    turn_rate = clamp_value(turn_rate, -kMaxTurnRateRadPerSec, kMaxTurnRateRadPerSec);

    // Convert heading and speed to target wheel speeds.
    motion_controller_set_wheel_targets(command->linear_speed - 0.5f * kTrackWidthM * turn_rate,
                                        command->linear_speed + 0.5f * kTrackWidthM * turn_rate);

    // Output to motor controller.

    set_open_loop_wheel_speed_targets(left_target, right_target);
}

void motion_controller_set_enabled(bool new_enabled)
{
    // Enable / disable motion controller
    enabled = new_enabled;
    if (!enabled) motion_controller_stop();
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

void motion_controller_set_wheel_targets(float left_speed, float right_speed)
{
    // Clamp values between max and min wheel speeds.
    left_target = clamp_value(left_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
    right_target = clamp_value(right_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
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
    motion_controller_stop(); // always start and end from a clean stop

    if (active)
    {
        float x, y, theta;
        get_ekf_pose(&x, &y, &theta);
        override_heading = theta; // hold the heading we were facing
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
        override_heading = theta; // re-capture so the next drive holds this heading
        reset_pid();
        motion_controller_set_wheel_targets(0.0f, 0.0f);
        set_open_loop_wheel_speed_targets(0.0f, 0.0f);
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
            const float heading_correction_limit =
                max_heading_correction_for_speed(speed_mps);
            turn = clamp_value(
                heading_pid_update(override_heading, theta, dt),
                -heading_correction_limit,
                heading_correction_limit);
        }
    }
    else
    {
        // Turning: command the rate directly and re-capture the heading
        turn = clamp_value(
            turn_rate_rad_s,
            -kMaxTurnRateRadPerSec,
            kMaxTurnRateRadPerSec);
        override_heading = theta;
        reset_pid();
    }

    motion_controller_set_wheel_targets(speed_mps - 0.5f * kTrackWidthM * turn,
                                        speed_mps + 0.5f * kTrackWidthM * turn);
    set_open_loop_wheel_speed_targets(left_target, right_target);
}
