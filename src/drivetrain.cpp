#include "drivetrain.h"
#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include <math.h>

// Modules handles the interface to the physical hardware, takes target wheel speeds and converts them to PPM signals for drivers

namespace
{
    constexpr uint8_t kLeftPwmPin = 28, kLeftENCA = 30, kLeftENCB = 31;
    constexpr uint8_t kRightPwmPin = 1, kRightENCA = 2, kRightENCB = 3;
    constexpr float kEncoderCpr = 3540.0f;
    constexpr float kWheelDiameterM = 0.075f;
    constexpr float kMetresPerCount = 3.14159265358979323846f * kWheelDiameterM / kEncoderCpr;
    constexpr int kStopUs = 1500, kPulseSpanUs = 450;
    constexpr float kMaxPwm = 255.0f;
    float output_limit_pwm = kMaxPwm;
    constexpr bool kLeftMotorInverted = true, kRightMotorInverted = false;
    constexpr bool kLeftEncoderInverted = false, kRightEncoderInverted = false;

    // Varying drivetrain friction, separate tuning for each motor
    struct Tuning
    {
        float kSForward, kSReverse, kVForward, kVReverse, kP, kI;
    };

    constexpr Tuning kLeftTuning = {0.0f, 0.0f, 850.0f, 850.0f, 450.0f, 500.0f};
    constexpr Tuning kRightTuning = {0.0f, 0.0f, 850.0f, 850.0f, 550.0f, 500.0f};
    constexpr float kAccelerationMps2 = 0.40f;
    constexpr float kDecelerationMps2 = 0.60f;
    constexpr float kPwmSlewPerSec = 1000.0f;
    constexpr float kSpeedFilterTimeConstantS = 0.030f;
    constexpr float kTargetDeadbandMps = 0.003f;
    constexpr uint32_t kMinimumUpdateUs = 5000;
    constexpr uint32_t kMaximumUpdateUs = 50000;
    constexpr uint32_t kReverseNeutralUs = 80000;
    constexpr float kReversalStoppedMps = 0.015f;
    constexpr uint32_t kStallUs = 3000000;
    constexpr uint32_t kWrongDirectionUs = 250000;


    // PI control for each motor to ensure motors turn at requested speeds.
    Servo left_driver, right_driver;
    Encoder left_encoder(kLeftENCA, kLeftENCB);
    Encoder right_encoder(kRightENCA, kRightENCB);

    struct Wheel
    {
        float requested = 0, reference = 0, measured = 0, integral = 0, output = 0;
        int32_t count = 0;
        int direction = 0;
        bool reversing = false;
        uint32_t reverse_start_us = 0, stalled_us = 0, wrong_direction_us = 0;
    };

    Wheel left, right;
    DrivetrainFault fault = DrivetrainFault::NONE;
    bool sampled = false;
    uint32_t previous_us = 0, last_interval_us = 0, max_interval_us = 0;

    // Helper functions
    float clampf(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

    float approach(float actual, float target, float step)
    {
        return actual + clampf(target - actual, -step, step);
    }

    void write_output(Servo& driver, bool inverted, float output)
    {
        const float logical = clampf(output, -kMaxPwm, kMaxPwm);
        const float physical = inverted ? -logical : logical;
        driver.writeMicroseconds(kStopUs + static_cast<int>(lroundf(physical * kPulseSpanUs / kMaxPwm)));
    }

    void reset_control(Wheel& w)
    {
        w.reference = w.integral = w.output = 0;
        w.stalled_us = w.wrong_direction_us = 0;
    }

    void stop_wheel(Wheel& w, Servo& driver)
    {
        w.requested = 0;
        reset_control(w);
        // Keep direction history so a subsequent reversal still waits for neutral/low speed.
        w.reversing = false;
        driver.writeMicroseconds(kStopUs);
    }

    void trip(DrivetrainFault reason)
    {
        if (fault == DrivetrainFault::NONE) fault = reason;
        drivetrain_stop();
    }

    void measure(Wheel& w, int32_t count, bool inverted, float dt)
    {
        // Measure wheel speed and apply filter to smooth.
        const uint32_t bits = static_cast<uint32_t>(count) - static_cast<uint32_t>(w.count);
        const int64_t delta = bits <= 0x7fffffffU
                                  ? static_cast<int64_t>(bits)
                                  : static_cast<int64_t>(bits) - 0x100000000LL;
        w.count = count;
        const float speed = (inverted ? -1.0f : 1.0f) * static_cast<float>(delta) * kMetresPerCount / dt;
        const float alpha = dt / (kSpeedFilterTimeConstantS + dt);
        w.measured += alpha * (speed - w.measured);
    }

    void control(Wheel& w, const Tuning& t, float dt, uint32_t dt_us, uint32_t now, DrivetrainFault stall_fault,
                 DrivetrainFault direction_fault)
    {
        // Apply the actual requested speed with the tuning values for the specific wheel to be driven.
        if (w.requested == 0)
        {
            reset_control(w);
            return;
        }
        const int direction = w.requested > 0 ? 1 : -1;
        if (w.direction != 0 && direction != w.direction)
        {
            w.reversing = true;
            w.reverse_start_us = now;
            reset_control(w);
        }
        w.direction = direction;
        if (w.reversing)
        {
            reset_control(w);
            if ((now - w.reverse_start_us) < kReverseNeutralUs ||
                fabsf(w.measured) > kReversalStoppedMps)
                return;
            w.reversing = false;
        }

        const bool accelerating = fabsf(w.requested) > fabsf(w.reference);
        w.reference = approach(w.reference, w.requested,
                               (accelerating ? kAccelerationMps2 : kDecelerationMps2) * dt);
        const float kS = direction > 0 ? t.kSForward : t.kSReverse;
        const float kV = direction > 0 ? t.kVForward : t.kVReverse;
        const float feedforward = direction * kS + kV * w.reference;
        const float error = w.reference - w.measured;

        // No opposite-drive braking while tracking: reduce toward neutral on overspeed.
        const float low = direction > 0 ? 0.0f : -output_limit_pwm;
        const float high = direction > 0 ? output_limit_pwm : 0.0f;
        const float slew_low = fmaxf(low, w.output - kPwmSlewPerSec * dt);
        const float slew_high = fminf(high, w.output + kPwmSlewPerSec * dt);
        const float candidate_integral = clampf(w.integral + t.kI * error * dt, -kMaxPwm, kMaxPwm);
        const float candidate = feedforward + t.kP * error + candidate_integral;

        // Conditional integration accounts for both output saturation and slew limiting.
        if ((candidate >= slew_low && candidate <= slew_high) ||
            (candidate > slew_high && error < 0) || (candidate < slew_low && error > 0))
        {
            w.integral = candidate_integral;
        }
        w.output = clampf(feedforward + t.kP * error + w.integral, slew_low, slew_high);
        // Check for stalls and incorrect directions.
        if (fabsf(w.reference) > 0.03f && fabsf(w.output) > 180.0f && fabsf(w.measured) < 0.005f)
            w.stalled_us += dt_us;
        else w.stalled_us = 0;
        if (direction * w.measured < -0.03f && fabsf(w.output) > 40.0f)
            w.wrong_direction_us += dt_us;
        else w.wrong_direction_us = 0;
        if (w.stalled_us >= kStallUs) trip(stall_fault);
        else if (w.wrong_direction_us >= kWrongDirectionUs) trip(direction_fault);
    }
} // namespace

void drivetrain_init()
{
    // Setup
    left_driver.attach(kLeftPwmPin, kStopUs - kPulseSpanUs, kStopUs + kPulseSpanUs);
    right_driver.attach(kRightPwmPin, kStopUs - kPulseSpanUs, kStopUs + kPulseSpanUs);
    drivetrain_stop();
    fault = DrivetrainFault::NONE;
    sampled = false; // First scheduled update establishes time/count baselines after GO.
    last_interval_us = max_interval_us = 0;
    output_limit_pwm = kMaxPwm;
}

void drivetrain_stop()
{
    stop_wheel(left, left_driver);
    stop_wheel(right, right_driver);
}

void drivetrain_clear_fault()
{
    drivetrain_stop();
    fault = DrivetrainFault::NONE;
    sampled = false;
}

DrivetrainFault drivetrain_get_fault() { return fault; }

void drivetrain_debug_task()
{
    Serial.printf(
        "DRIVE L req=%.3f ref=%.3f vel=%.3f pwm=%.0f rev=%u | "
        "R req=%.3f ref=%.3f vel=%.3f pwm=%.0f rev=%u | "
        "fault=%u dt=%lu max=%lu\n",
        left.requested,
        left.reference,
        left.measured,
        left.output,
        left.reversing ? 1u : 0u,
        right.requested,
        right.reference,
        right.measured,
        right.output,
        right.reversing ? 1u : 0u,
        static_cast<unsigned>(fault),
        static_cast<unsigned long>(last_interval_us),
        static_cast<unsigned long>(max_interval_us));
}

void set_wheel_speed_targets(float l, float r)
{
    if (!isfinite(l) || !isfinite(r))
    {
        trip(DrivetrainFault::INVALID_TARGET);
        return;
    }
    if (fault != DrivetrainFault::NONE)
    {
        drivetrain_stop();
        return;
    }
    // Preserve wheel ratio if callers supply a target beyond the speed limit.
    const float peak = fmaxf(fabsf(l), fabsf(r));
    if (peak > kDrivetrainMaxWheelSpeedMps)
    {
        const float scale = kDrivetrainMaxWheelSpeedMps / peak;
        l *= scale;
        r *= scale;
    }
    left.requested = fabsf(l) < kTargetDeadbandMps ? 0 : l;
    right.requested = fabsf(r) < kTargetDeadbandMps ? 0 : r;
    // A zero command always reaches neutral immediately, even between update ticks.
    if (left.requested == 0) stop_wheel(left, left_driver);
    if (right.requested == 0) stop_wheel(right, right_driver);
}

void drivetrain_update()
{
    const uint32_t now = micros();
    if (!sampled)
    {
        left.count = left_encoder.read();
        right.count = right_encoder.read();
        left.measured = right.measured = 0;
        previous_us = now;
        sampled = true;
        return;
    }
    const uint32_t elapsed = now - previous_us;
    if (elapsed < kMinimumUpdateUs) return;
    previous_us = now;
    last_interval_us = elapsed;
    if (elapsed > max_interval_us) max_interval_us = elapsed;
    const float dt = elapsed * 1.0e-6f;
    measure(left, left_encoder.read(), kLeftEncoderInverted, dt);
    measure(right, right_encoder.read(), kRightEncoderInverted, dt);
    if (elapsed > kMaximumUpdateUs)
    {
        //trip(DrivetrainFault::CONTROL_OVERRUN);
        return;
    }
    if (fault != DrivetrainFault::NONE)
    {
        drivetrain_stop();
        return;
    }
    control(left, kLeftTuning, dt, elapsed, now,
            DrivetrainFault::LEFT_STALLED, DrivetrainFault::LEFT_ENCODER_DIRECTION);
    if (fault != DrivetrainFault::NONE) return;
    control(right, kRightTuning, dt, elapsed, now,
            DrivetrainFault::RIGHT_STALLED, DrivetrainFault::RIGHT_ENCODER_DIRECTION);
    if (fault != DrivetrainFault::NONE) return;
    write_output(left_driver, kLeftMotorInverted, left.output);
    write_output(right_driver, kRightMotorInverted, right.output);
}

void drivetrain_get_telemetry(DrivetrainTelemetry* t)
{
    if (!t) return;
    *t = {
        left.requested, right.requested, left.reference, right.reference,
        left.measured, right.measured,
        static_cast<int16_t>(lroundf(left.output)), static_cast<int16_t>(lroundf(right.output)),
        left.count, right.count, last_interval_us, max_interval_us, fault
    };
}

void drivetrain_set_output_limit(int maximum_pwm)
{
    const float limit = clampf(
        static_cast<float>(maximum_pwm), 1.0f, kMaxPwm);

    if (limit == output_limit_pwm)
    {
        return;
    }

    output_limit_pwm = limit;
    drivetrain_stop(); // Clear old targets, PWM and integral before changing modes.
}
