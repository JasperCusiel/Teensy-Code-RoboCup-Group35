//
// Created by Jasper Cusiel on 06/09/2026.
//

#include "drivetrain.h"

#include <Arduino.h>
#include <Servo.h>

namespace {

constexpr uint8_t kLeftPwmPin = 28;
constexpr uint8_t kRightPwmPin = 1;

constexpr int kFullForwardUs = 1950;
constexpr int kFullReverseUs = 1050;
constexpr int kStopUs = 1500;

Servo left_motor;
Servo right_motor;

float clamp_output(float output) {
  return fminf(1.0f, fmaxf(-1.0f, output));
}

int output_to_pulse(float output) {
  const float clamped = clamp_output(output);
  const float pulse = clamped >= 0.0f
                          ? kStopUs + clamped * (kFullForwardUs - kStopUs)
                          : kStopUs + clamped * (kStopUs - kFullReverseUs);
  return static_cast<int>(lroundf(pulse));
}

} // namespace

void drivetrain_init() {
  left_motor.attach(kLeftPwmPin, kFullReverseUs, kFullForwardUs);
  right_motor.attach(kRightPwmPin, kFullReverseUs, kFullForwardUs);
  set_motor_speeds(0.0f, 0.0f);
}

void set_motor_speeds(float left, float right) {
  left_motor.writeMicroseconds(output_to_pulse(left));
  right_motor.writeMicroseconds(output_to_pulse(right));
}
