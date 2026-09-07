//
// Created by Jasper Cusiel on 06/09/2026.
//

#include "drivetrain.h"

#include <Arduino.h>
#include <Servo.h>

namespace
{
    // Motor drive config
    constexpr uint8_t kLeftPwmPin = 28;
    constexpr uint8_t kRightPwmPin = 1;

    constexpr int kFullForwardUs = 1950;
    constexpr int kFullReverseUs = 1050;
    constexpr int kStopUs = 1500;

    // Motor driver uses PPM control (essentially the same as a servo)
    Servo left_motor;
    Servo right_motor;
 // Used to clamp the control output signal
    float clamp_output(float output)
    {
        return fminf(1.0f, fmaxf(-1.0f, output));
    }

    // Converts control output into pulse range for motor driver
    int output_to_pulse(float output)
    {
        const float clamped = clamp_output(output);
        const float pulse = clamped >= 0.0f
                                ? kStopUs + clamped * (kFullForwardUs - kStopUs)
                                : kStopUs + clamped * (kStopUs - kFullReverseUs);
        return static_cast<int>(lroundf(pulse));
    }
} // namespace

void drivetrain_init()
{
    // Initialize motor drives
    left_motor.attach(kLeftPwmPin, kFullReverseUs, kFullForwardUs);
    right_motor.attach(kRightPwmPin, kFullReverseUs, kFullForwardUs);

    // Make sure we don't command any motor speed on startup.
    set_motor_speeds(0.0f, 0.0f);
}

void set_motor_speeds(float left, float right)
{
    // Convert commanded wheel speeds to motor drive pulse commands.
    left_motor.writeMicroseconds(output_to_pulse(left));
    right_motor.writeMicroseconds(output_to_pulse(right));
}
