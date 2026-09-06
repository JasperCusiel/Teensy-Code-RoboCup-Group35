//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "motion-controller.h"

#include "odometry.h"

#include <Arduino.h>
#include <math.h>

namespace {

constexpr float kTrackWidthM = 0.28f;
constexpr float kMaxWheelSpeedMps = 0.50f;
constexpr float kHeadingKp = 0.5f;
constexpr float kHeadingKi = 0.00f;
constexpr float kHeadingKd = 0.00f;
constexpr float kSpeedKp = 1.0f;
constexpr float kSpeedKi = 0.0f;
constexpr float kSpeedKd = 0.00f;
constexpr float kMaxMotorEffort = 1.0f;

bool enabled = false;
drive_output_callback_t output_callback = nullptr;
float left_target = 0.0f;
float right_target = 0.0f;
float left_output = 0.0f;
float right_output = 0.0f;
float measured_forward_speed = 0.0f;
float heading_integral = 0.0f;
float heading_previous_error = 0.0f;
float speed_integral = 0.0f;
float speed_previous_error = 0.0f;
uint32_t previous_update_us = 0;

float clamp_value(float value, float minimum, float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

float wrap_angle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

void reset_pid() {
  heading_integral = 0.0f;
  heading_previous_error = 0.0f;
  speed_integral = 0.0f;
  speed_previous_error = 0.0f;
}

} // namespace

void motion_controller_init() {
  enabled = true;
  previous_update_us = micros();
  reset_pid();
  motion_controller_stop();
}

void motion_controller_stop() {
  left_target = 0.0f;
  right_target = 0.0f;
  left_output = 0.0f;
  right_output = 0.0f;
  reset_pid();
  motion_controller_apply_motor_output(0.0f, 0.0f);
}

void motion_controller_update(const pose_t *pose,
                              const velocity_command_t *command) {
  const uint32_t now = micros();
  float dt = static_cast<float>(now - previous_update_us) * 1.0e-6f;
  previous_update_us = now;
  if (dt <= 0.0f || dt > 0.25f) dt = 1.0f / 95.0f;

  if (!enabled || pose == nullptr || command == nullptr || command->stop) {
    motion_controller_stop();
    return;
  }

  float gyro_z = 0.0f;
  float measured_heading = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f;
  get_sensor_data(&gyro_z, &measured_heading, &vx, &vy);
  (void)gyro_z;
  (void)measured_heading;
  (void)vx;
  measured_forward_speed = vy;

  const float turn_rate = heading_pid_update(command->heading, pose->theta, dt) +
                          command->turn_rate;
  const float drive_effort = speed_pid_update(command->linear_speed,
                                               measured_forward_speed, dt);

  motion_controller_set_wheel_targets(
      command->linear_speed - 0.5f * kTrackWidthM * turn_rate,
      command->linear_speed + 0.5f * kTrackWidthM * turn_rate);

  const float turn_effort = 0.5f * kTrackWidthM * turn_rate /
                            kMaxWheelSpeedMps;
  left_output = clamp_value(drive_effort - turn_effort, -kMaxMotorEffort, kMaxMotorEffort);

  right_output = clamp_value(drive_effort + turn_effort,-kMaxMotorEffort,kMaxMotorEffort);

  motion_controller_apply_motor_output(left_output, right_output);
}

void motion_controller_set_enabled(bool new_enabled) {
  enabled = new_enabled;
  if (!enabled) motion_controller_stop();
}

void motion_controller_set_output_callback(drive_output_callback_t callback) {
  output_callback = callback;
}

float heading_pid_update(float target_heading, float current_heading, float dt) {
  const float error = wrap_angle(target_heading - current_heading);
  heading_integral = clamp_value(heading_integral + error * dt, -1.0f, 1.0f);
  const float derivative = (error - heading_previous_error) / dt;
  heading_previous_error = error;
  return kHeadingKp * error + kHeadingKi * heading_integral +
         kHeadingKd * derivative;
}

float speed_pid_update(float target_speed, float current_speed, float dt) {
  const float error = target_speed - current_speed;
  speed_integral = clamp_value(speed_integral + error * dt, -1.0f, 1.0f);
  const float derivative = (error - speed_previous_error) / dt;
  speed_previous_error = error;
  return clamp_value(kSpeedKp * error + kSpeedKi * speed_integral +
                         kSpeedKd * derivative,
                     -1.0f, 1.0f);
}

void motion_controller_set_wheel_targets(float left_speed, float right_speed) {
  left_target = clamp_value(left_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
  right_target = clamp_value(right_speed, -kMaxWheelSpeedMps, kMaxWheelSpeedMps);
}

void motion_controller_apply_motor_output(float left, float right) {
  if (output_callback != nullptr) output_callback(left, right);
}

void motion_controller_get_wheel_targets(float *left_speed, float *right_speed) {
  if (left_speed != nullptr) *left_speed = left_target;
  if (right_speed != nullptr) *right_speed = right_target;
}

void motion_controller_get_outputs(float *left, float *right) {
  if (left != nullptr) *left = left_output;
  if (right != nullptr) *right = right_output;
}

bool robot_is_stopped() { return fabsf(measured_forward_speed) < 0.03f; }
