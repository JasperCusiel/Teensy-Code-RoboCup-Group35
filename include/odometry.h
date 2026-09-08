//
// Created by Jasper Cusiel on 28/07/2026.
//

#ifndef ROBOCUP_ODOMETRY_H
#define ROBOCUP_ODOMETRY_H

typedef struct
{
    double gyro_z;
    double heading;
    double vx_meas;
    double vy_meas;
} sensor_data_t;

typedef struct
{
    float x;
    float y;
    float theta;
} pose_t;

// Start odom and set initial position based on which base we are on.
void odometry_init();

// Call periodically to update EKF position estimate.
void odometry_update();

// Getter functions
void get_ekf_pose(float* x, float* y, float* theta);
void get_sensor_data(double* gyro_z, double* heading, double* vx_meas, double* vy_meas);
double get_vy();

#endif // ROBOCUP_ODOMETRY_H
