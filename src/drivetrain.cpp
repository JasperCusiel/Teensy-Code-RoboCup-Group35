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
    constexpr int kPWM_MAX = 255;
    constexpr float kOpenLoopDeadbandMps = 0.005f;
    constexpr uint8_t kPwmSkip = 170;
    constexpr uint8_t kLeftSkip = 0;
    constexpr uint8_t kRightSkip = 60;


    template <Servo& Motor, bool MotorInverted>
    struct MotorCallbacks
    {
        static void write(int16_t speed)
        {
            const int16_t driver_speed = MotorInverted ? -speed : speed;
            const int pulse = map(driver_speed, -kPWM_MAX, kPWM_MAX, kFullReverseUs, kFullForwardUs);
            Motor.writeMicroseconds(pulse);
        }

        static void brake() { Motor.writeMicroseconds(kStopUs); }
    };

    // Motor driver uses PPM control (essentially the same as a servo)
    // Motor drives are PPM controlled (thus the Servo library generates pulses).
    Servo left_driver;
    Servo right_driver;

    using Motor1Callbacks = MotorCallbacks<left_driver, true>;
    using Motor2Callbacks = MotorCallbacks<right_driver, false>;


    int16_t wheel_speed_to_pwm(float wheel_speed_mps, int base_skip)
    {
        if (wheel_speed_mps > kDrivetrainMaxWheelSpeedMps)
        {
            wheel_speed_mps = kDrivetrainMaxWheelSpeedMps;
        }
        else if (wheel_speed_mps < -kDrivetrainMaxWheelSpeedMps)
        {
            wheel_speed_mps = -kDrivetrainMaxWheelSpeedMps;
        }

        const bool reverse = wheel_speed_mps < 0.0f;
        const float magnitude_mps = reverse ? -wheel_speed_mps : wheel_speed_mps;
        if (magnitude_mps < kOpenLoopDeadbandMps)
        {
            return 0;
        }

        const float normalized = magnitude_mps / kDrivetrainMaxWheelSpeedMps;
        const int pwm = (base_skip + kPwmSkip) + static_cast<int>((kPWM_MAX - (kPwmSkip + base_skip)) * normalized);
        return reverse ? (-pwm) : pwm;
    }
} // namespace

void set_open_loop_wheel_speed_targets(float left_mps, float right_mps)
{
    if (fabsf(left_mps) < kOpenLoopDeadbandMps)
    {
        Motor1Callbacks::brake();
    }
    else
    {
        Motor1Callbacks::write(wheel_speed_to_pwm(left_mps, kLeftSkip));
    }

    if (fabsf(right_mps) < kOpenLoopDeadbandMps)
    {
        Motor2Callbacks::brake();
    }
    else
    {
        Motor2Callbacks::write(wheel_speed_to_pwm(right_mps, kRightSkip));
    }
    // Serial.printf(
    //     "FINAL: L=%.3f R=%.3f\n",
    //     left_mps,
    //     right_mps
    // );
}

void drivetrain_init()
{
    // Initialize motor drives
    left_driver.attach(kLeftPwmPin, kFullReverseUs, kFullForwardUs);
    right_driver.attach(kRightPwmPin, kFullReverseUs, kFullForwardUs);

    // Make sure we don't command any motor speed on startup.
    set_open_loop_wheel_speed_targets(0.0f, 0.0f);
}
