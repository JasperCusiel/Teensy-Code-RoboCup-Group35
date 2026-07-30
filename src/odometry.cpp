//
// Created by Jasper Cusiel on 28/07/2026.
//

#define EKF_N 5
#define EKF_M 3

#define ODOM_FREQ 95
#define DT (1.0f/ ODOM_FREQ)

#include "odometry.h"
#include "fl/math_macros.h"
#include "imu.h"
#include "optical-flow.h"
#include "tinyekf.h"
#include <arduino.h>


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


void odometry_init() {
  // Use identity matrix as initial covariance matrix

  const float Pdiag[EKF_N] = {
    1, 1, 0.5, 1, 1};
  ekf_initialize(&_ekf, Pdiag);
}



void update_F(float dt, float vx, float vy)
{
  float theta = _ekf.x[2];
  float c = cosf(theta);
  float s = sinf(theta);

  memset(F, 0, sizeof(F));

  F[0*EKF_N+0] = 1.0f;
  F[0*EKF_N+2] = (-vx*s - vy*c) * dt;
  F[0*EKF_N+3] = c * dt;
  F[0*EKF_N+4] = -s * dt;

  F[1*EKF_N+1] = 1.0f;
  F[1*EKF_N+2] = (vx*c - vy*s) * dt;
  F[1*EKF_N+3] = s * dt;
  F[1*EKF_N+4] = c * dt;

  F[2*EKF_N+2] = 1.0f;
  F[3*EKF_N+3] = 1.0f;
  F[4*EKF_N+4] = 1.0f;
}


void odometry_update() {
  float gyro_z = imu_get_gyro_z();
  float heading = imu_get_heading();

  float vx_meas, vy_meas;
  flow_get_velocity(&vx_meas, &vy_meas, DT);


  _sensor_data.gyro_z = gyro_z;
  _sensor_data.heading = heading;
  _sensor_data.vx_meas = vx_meas;
  _sensor_data.vy_meas = vy_meas;

  compensate_flow(&vx_meas, &vy_meas, gyro_z);

  float theta = _ekf.x[2];
  float c = cosf(theta);
  float s = sinf(theta);

  // Use STATE velocity for prediction
  float vx_world = _ekf.x[3] * c - _ekf.x[4] * s;
  float vy_world = _ekf.x[3] * s + _ekf.x[4] * c;

  float predicted_heading = theta + gyro_z * DT;

  // Wrap
  while (predicted_heading > PI) predicted_heading -= 2 * PI;
  while (predicted_heading < -PI) predicted_heading += 2 * PI;

  float fx[EKF_N] = {
    _ekf.x[0] + vx_world * DT,
    _ekf.x[1] + vy_world * DT,
    predicted_heading,
    _ekf.x[3],
    _ekf.x[4]
  };

  update_F(DT, _ekf.x[3], _ekf.x[4]);
  ekf_predict(&_ekf, fx, F, Q);

  // Wrap heading measurement
  float heading_error = heading - _ekf.x[2];
  while (heading_error > PI) heading_error -= 2 * PI;
  while (heading_error < -PI) heading_error += 2 * PI;

  float wrapped_heading = _ekf.x[2] + heading_error;

  float hx[EKF_M] = {
    _ekf.x[3],
    _ekf.x[4],
    _ekf.x[2]
  };

  float z[EKF_M] = {
    vx_meas,
    vy_meas,
    wrapped_heading
  };

  ekf_update(&_ekf, z, hx, H, R);
}

void print_ekf_pose() {
  Serial.printf("x=%.2f y=%.2f heading=%.2f\n", _ekf.x[0], _ekf.x[1], degrees(_ekf.x[2]));
}

void get_ekf_pose(float *x, float* y, float* theta) {
  *x = _ekf.x[0];
  *y = _ekf.x[1];
  *theta = _ekf.x[2];
}

void get_sensor_data(float* gyro_z, float* heading, float* vx_meas, float* vy_meas) {
  *gyro_z = _sensor_data.gyro_z;
  *heading = _sensor_data.heading;
  *vx_meas = _sensor_data.vx_meas;
  *vy_meas = _sensor_data.vy_meas;
}

