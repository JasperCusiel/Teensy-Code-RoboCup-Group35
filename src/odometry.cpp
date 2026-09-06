//
// Created by Jasper Cusiel on 28/07/2026.
//

#define EKF_N 5
#define EKF_M 3

#define BLUE_BASE_X 2.8f
#define GREEN_BASE_X 0.2f
#define BASE_Y  0.2f

#define X 0
#define Y 1
#define THETA 2
#define BODY_VX 3
#define BODY_VY 4

#define ODOM_FREQ 95
#define NOMINAL_DT (1.0f / ODOM_FREQ)
#define MIN_ODOM_DT (0.5f * NOMINAL_DT)

#include "odometry.h"
#include "fl/math_macros.h"
#include "imu.h"
#include "optical-flow.h"
#include "tinyekf.h"
#include <arduino.h>
#include "colour-sensor.h"
#include "navigation.h"


// Proccess noise covariance
static const float Q[EKF_N*EKF_N] = {
  0.01, 0,   0,   0,   0,
  0,   0.01, 0,   0,   0,
  0,   0,   0.001, 0,   0,
  0,   0,   0,   0.1, 0,
  0,   0,   0,   0,   0.1
};

// Measurement noise covarance
static const float R[EKF_M*EKF_M] = {
  0.05, 0,   0,
  0,   0.05, 0,
  0,   0,   0.02
};

// Process model
float F[EKF_N*EKF_N];


// Measurement matrix H
//
// Measurements:
// vx  = state[3]
// vy  = state[4]
// heading = state[2]
static const float H[EKF_M*EKF_N] = {
  0, 0, 0, 1, 0,
  0, 0, 0, 0, 1,
  0, 0, 1, 0, 0
};

static ekf_t _ekf;
static sensor_data _sensor_data;
static uint32_t _last_update_us = 0;


void odometry_init() {
  // Use identity matrix as initial covariance matrix

  const float Pdiag[EKF_N] = {
    1, 1, 0.5, 1, 1};
  ekf_initialize(&_ekf, Pdiag);
  float initial_x = (get_base_color() == COLOR_GREEN) ? GREEN_BASE_X : BLUE_BASE_X;
  float initial_theta = 0;
  navigation_set_base(initial_x, BASE_Y);
  _ekf.x[X] = initial_x;
  _ekf.x[Y] = BASE_Y;
  _ekf.x[THETA] = initial_theta;
  _ekf.x[BODY_VX] = 0.0f;
  _ekf.x[BODY_VY] = 0.0f;
  _last_update_us = 0;

}



void update_F(float dt, float vx, float vy)
{
  float theta = _ekf.x[THETA];
  float c = cosf(theta);
  float s = sinf(theta);

  memset(F, 0, sizeof(F));

  F[0*EKF_N + 0] = 1.0f;
  F[0*EKF_N + 2] = (-vx*s - vy*c) * dt;
  F[0*EKF_N + 3] = c * dt;
  F[0*EKF_N + 4] = -s * dt;

  F[1*EKF_N + 1] = 1.0f;
  F[1*EKF_N + 2] = (vx*c - vy*s) * dt;
  F[1*EKF_N + 3] = s * dt;
  F[1*EKF_N + 4] = c * dt;

  // heading
  F[2*EKF_N + 2] = 1.0f;

  // velocity
  F[3*EKF_N + 3] = 1.0f;
  F[4*EKF_N + 4] = 1.0f;
}


void odometry_update() {
  // Use the interval between optical-flow reads rather than DT
  if (_last_update_us == 0) {
    float discarded_vx, discarded_vy;
    flow_get_velocity(&discarded_vx, &discarded_vy, NOMINAL_DT);
    _last_update_us = micros();
    return;
  }

  const uint32_t now_us = micros();
  const float dt = static_cast<uint32_t>(now_us - _last_update_us) * 1.0e-6f;

  // Avoids dividing by tiny count if scheduler overrun happens
  if (dt < MIN_ODOM_DT) {
    return;
  }
  _last_update_us = now_us;

  float gyro_z = imu_get_gyro_z();
  float heading = imu_get_heading();

  float vx_meas, vy_meas;
  flow_get_velocity(&vx_meas, &vy_meas, dt);


  _sensor_data.gyro_z = gyro_z;
  _sensor_data.heading = heading;
  _sensor_data.vx_meas = vx_meas;
  _sensor_data.vy_meas = vy_meas;

  compensate_flow(&vx_meas, &vy_meas, gyro_z);

  float theta = _ekf.x[THETA];
  float c = cosf(theta);
  float s = sinf(theta);

  // Use STATE velocity for prediction
  // +X = right
  // +Y = forward
  // +theta = counter-clockwise about +Z

  float vx_world = _ekf.x[BODY_VX] * c - _ekf.x[BODY_VY] * s;
  float vy_world = _ekf.x[BODY_VX] * s + _ekf.x[BODY_VY] * c;

  float predicted_heading = theta + gyro_z * dt;

  // Wrap
  while (predicted_heading > PI) predicted_heading -= 2 * PI;
  while (predicted_heading < -PI) predicted_heading += 2 * PI;

  float fx[EKF_N] = {
    _ekf.x[X] + vx_world * dt,
    _ekf.x[Y] + vy_world * dt,
    predicted_heading,
    _ekf.x[BODY_VX],
    _ekf.x[BODY_VY]
  };

  update_F(dt, _ekf.x[BODY_VX], _ekf.x[BODY_VY]);
  ekf_predict(&_ekf, fx, F, Q);

  // Wrap heading measurement
  float heading_error = heading - _ekf.x[2];
  while (heading_error > PI) heading_error -= 2 * PI;
  while (heading_error < -PI) heading_error += 2 * PI;

  float wrapped_heading = _ekf.x[2] + heading_error;

  float hx[EKF_M] = {
    _ekf.x[BODY_VX],
    _ekf.x[BODY_VY],
    _ekf.x[THETA]
  };

  float z[EKF_M] = {
    vx_meas,
    vy_meas,
    wrapped_heading
  };

  ekf_update(&_ekf, z, hx, H, R);
}

void print_ekf_pose() {
  Serial.printf("x=%.2f y=%.2f heading=%.2f\n", _ekf.x[X], _ekf.x[Y], degrees(_ekf.x[THETA]));
}

void get_ekf_pose(float *x, float* y, float* theta) {
  *x = _ekf.x[X];
  *y = _ekf.x[Y];
  *theta = _ekf.x[THETA];
}

void get_sensor_data(float* gyro_z, float* heading, float* vx_meas, float* vy_meas) {
  *gyro_z = _sensor_data.gyro_z;
  *heading = _sensor_data.heading;
  *vx_meas = _sensor_data.vx_meas;
  *vy_meas = _sensor_data.vy_meas;
}
