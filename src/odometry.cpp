//
// Created by Jasper Cusiel on 28/07/2026.
//

#define EKF_N 5
#define EKF_M 3

#include "odometry.h"
#include "fl/math_macros.h"
#include "imu.h"
#include "optical-flow.h"
#include "tinyekf.h"

static const float EPS = 1e-4;

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


void odometry_update(float dt) {
  float heading = imu_get_heading();
  float gyro_z = imu_get_gyro_z();
  float vx, vy;
  flow_get_velocity(&vx, &vy);
  compensate_flow(&vx, &vy, gyro_z);


  // Prediction model
  float theta = _ekf.x[2];
  float c = cosf(theta);
  float s = sinf(theta);

  // Convert body velocity to world velocity
  float vx_world = _ekf.x[3] * c - _ekf.x[4] * s;
  float vy_world = _ekf.x[3] * s + _ekf.x[4] * c;

  float predicted_heading = _ekf.x[2] + gyro_z * dt;

  while (predicted_heading > PI)
    predicted_heading -= 2 * PI;

  while (predicted_heading < -PI)
    predicted_heading += 2 * PI;


  float fx[EKF_N] = {
    _ekf.x[0] + vx_world * dt,
    _ekf.x[1] + vy_world * dt,
    predicted_heading,
    vx,
    vy
};

  // Process model:
  // x,y updated from body velocity
  // heading updated from gyro
  // velocity updated from optical flow
  update_F(dt, vx, vy);

  // Run the prediction step of the DKF
  ekf_predict(&_ekf, fx, F, Q);

  // Wrap measurement relative to predicted state
  float heading_error = heading - _ekf.x[2];

  while (heading_error > PI)
    heading_error -= 2 * PI;

  while (heading_error < -PI)
    heading_error += 2 * PI;

  float wrapped_heading = _ekf.x[2] + heading_error;


  // Expected measurements from state
  float hx[EKF_M] = {
    _ekf.x[3],  // predicted vx
    _ekf.x[4],  // predicted vy
    _ekf.x[2]   // predicted heading
  };

  // Set the observation vector z (measurements)
  // [body vx, body vy, heading]
  float z[EKF_M] = {vx, vy, wrapped_heading};

  // Run the update step
  ekf_update(&_ekf, z, hx, H, R);
}

void get_ekf_pose(float *out) {
  for (int i = 0; i < EKF_N; i++) {
    out[i] = _ekf.x[i];
  }
}

