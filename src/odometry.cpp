//
// Created by Jasper Cusiel on 28/07/2026.
//

// These must be defined before #include "tinyekf.h"
#define EKF_N 5
#define EKF_M 3

#include "odometry.h"
#include "fl/math_macros.h"
#include "imu.h"
#include "optical-flow.h"
#include "tinyekf.h"
#include <arduino.h>
#include "colour-sensor.h"
#include "navigation.h"

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

// This module updates the EKF based on the optical flow and imu measurements.

// Process noise covariance
static const float Q[EKF_N * EKF_N] = {
    0.01, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0,
    0, 0, 0.001, 0, 0,
    0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0.1
};

// Measurement noise covariance
static const float R[EKF_M * EKF_M] = {
    0.05, 0, 0,
    0, 0.05, 0,
    0, 0, 0.02
};

// Process model
float F[EKF_N * EKF_N];


// Measurement matrix H
//
// Measurements:
// vx  = state[3]
// vy  = state[4]
// heading = state[2]
static const float H[EKF_M * EKF_N] = {
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1,
    0, 0, 1, 0, 0
};

static ekf_t ekf;
static sensor_data_t sensor_data;
static uint32_t last_update_us = 0;


void odometry_init()
{
    // Use identity matrix as initial covariance matrix
    const float Pdiag[EKF_N] = {1, 1, 0.5, 1, 1};
    ekf_initialize(&ekf, Pdiag);
    float initial_x = (get_base_color() == COLOR_GREEN) ? GREEN_BASE_X : BLUE_BASE_X;
    float initial_theta = 0;

    navigation_set_base(initial_x, BASE_Y);

    // Set initial EKF position based on which base we are on.
    ekf.x[X] = initial_x;
    ekf.x[Y] = BASE_Y;
    ekf.x[THETA] = initial_theta;
    ekf.x[BODY_VX] = 0.0f;
    ekf.x[BODY_VY] = 0.0f;
    last_update_us = 0;
}


void update_F(float dt, float vx, float vy)
{
    // Update state transition matrix.
    float theta = ekf.x[THETA];
    float c = cosf(theta);
    float s = sinf(theta);

    memset(F, 0, sizeof(F));

    F[0 * EKF_N + 0] = 1.0f;
    F[0 * EKF_N + 2] = (-vx * s - vy * c) * dt;
    F[0 * EKF_N + 3] = c * dt;
    F[0 * EKF_N + 4] = -s * dt;

    F[1 * EKF_N + 1] = 1.0f;
    F[1 * EKF_N + 2] = (vx * c - vy * s) * dt;
    F[1 * EKF_N + 3] = s * dt;
    F[1 * EKF_N + 4] = c * dt;

    // heading
    F[2 * EKF_N + 2] = 1.0f;

    // velocity
    F[3 * EKF_N + 3] = 1.0f;
    F[4 * EKF_N + 4] = 1.0f;
}


void odometry_update()
{
    // Function samples the sensors and update the EKF.

    // Use the interval between optical-flow reads rather than DT
    if (last_update_us == 0)
    {
        float discarded_vx, discarded_vy;
        flow_get_velocity(&discarded_vx, &discarded_vy, NOMINAL_DT);
        last_update_us = micros();
        return;
    }

    const uint32_t now_us = micros();
    const float dt = static_cast<uint32_t>(now_us - last_update_us) * 1.0e-6f;

    // Avoids dividing by tiny count if scheduler overrun happens
    if (dt < MIN_ODOM_DT)
    {
        return;
    }
    last_update_us = now_us;

    // Get latest sensor data
    float gyro_z = imu_get_gyro_z();
    float heading = imu_get_heading();

    float vx_meas, vy_meas;
    flow_get_velocity(&vx_meas, &vy_meas, dt);

    // Store to allow other modules to access via getter functions.
    sensor_data.gyro_z = gyro_z;
    sensor_data.heading = heading;
    sensor_data.vx_meas = vx_meas;
    sensor_data.vy_meas = vy_meas;

    // Compensate for optical flow sensor physical offset from robot center.
    compensate_flow(&vx_meas, &vy_meas, gyro_z);

    float theta = ekf.x[THETA];
    float c = cosf(theta);
    float s = sinf(theta);

    // Use STATE velocity for prediction (right hand coordinate system)
    // +X = right
    // +Y = forward
    // +theta = counter-clockwise about +Z

    float vx_world = ekf.x[BODY_VX] * c - ekf.x[BODY_VY] * s;
    float vy_world = ekf.x[BODY_VX] * s + ekf.x[BODY_VY] * c;

    float predicted_heading = theta + (float)gyro_z * dt;

    // Wrap heading
    while (predicted_heading > PI) predicted_heading -= 2 * PI;
    while (predicted_heading < -PI) predicted_heading += 2 * PI;

    float fx[EKF_N] = {
        ekf.x[X] + vx_world * dt,
        ekf.x[Y] + vy_world * dt,
        predicted_heading,
        ekf.x[BODY_VX],
        ekf.x[BODY_VY]
    };

    update_F(dt, ekf.x[BODY_VX], ekf.x[BODY_VY]);
    ekf_predict(&ekf, fx, F, Q);

    // Wrap heading measurement
    float heading_error = (float)heading - ekf.x[2];
    while (heading_error > PI) heading_error -= 2 * PI;
    while (heading_error < -PI) heading_error += 2 * PI;

    float wrapped_heading = ekf.x[2] + heading_error;

    float hx[EKF_M] = {
        ekf.x[BODY_VX],
        ekf.x[BODY_VY],
        ekf.x[THETA]
    };

    float z[EKF_M] = {(float)vx_meas, (float)vy_meas, wrapped_heading};

    ekf_update(&ekf, z, hx, H, R);
}


void get_ekf_pose(float* x, float* y, float* theta)
{
    *x = ekf.x[X];
    *y = ekf.x[Y];
    *theta = ekf.x[THETA];
}

void get_sensor_data(float* gyro_z, float* heading, float* vx_meas, float* vy_meas)
{
    *gyro_z = sensor_data.gyro_z;
    *heading = sensor_data.heading;
    *vx_meas = sensor_data.vx_meas;
    *vy_meas = sensor_data.vy_meas;
}

float get_vy()
{
    return sensor_data.vy_meas;
}
