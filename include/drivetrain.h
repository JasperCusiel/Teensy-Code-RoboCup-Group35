#ifndef ROBOCUP_DRIVETRAIN_H
#define ROBOCUP_DRIVETRAIN_H
#include <stdint.h>

constexpr float kDrivetrainMaxWheelSpeedMps = 0.30f;


enum class DrivetrainFault : uint8_t
{
    NONE, CONTROL_OVERRUN, INVALID_TARGET, LEFT_STALLED, RIGHT_STALLED,
    LEFT_ENCODER_DIRECTION, RIGHT_ENCODER_DIRECTION
};

struct DrivetrainTelemetry
{
    float left_requested_mps, right_requested_mps;
    float left_ramped_mps, right_ramped_mps;
    float left_measured_mps, right_measured_mps;
    int16_t left_pwm, right_pwm; // Logical forward-positive, before motor inversion.
    int32_t left_counts, right_counts; // Raw encoder counts, before encoder inversion.
    uint32_t last_interval_us, max_interval_us;
    DrivetrainFault fault;
};

void drivetrain_init();
void drivetrain_update(); // Schedule at 100 Hz; never call from an ISR.
void drivetrain_stop(); // Immediate neutral, also clears requested targets.
void drivetrain_clear_fault(); // Call only while stopped, after fixing the cause.
DrivetrainFault drivetrain_get_fault();
void drivetrain_get_telemetry(DrivetrainTelemetry* telemetry);
void drivetrain_debug_task();
void drivetrain_set_output_limit(int maximum_pwm);

// Name retained to avoid changes in existing motion-controller.cpp.
// This now sets CLOSED-LOOP velocity targets; drivetrain_update drives the motors.
void set_open_loop_wheel_speed_targets(float left_mps, float right_mps);
#endif
