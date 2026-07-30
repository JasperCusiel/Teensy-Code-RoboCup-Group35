//
// Created by Jasper Cusiel on 28/07/2026.
//

#ifndef ROBOCUP_ODOMETRY_H
#define ROBOCUP_ODOMETRY_H

typedef struct {
  float gyro_z;
  float heading;
  float vx_meas;
  float vy_meas;
} sensor_data;


void odometry_init();
void odometry_update();
void print_ekf_pose();
void get_ekf_pose(float *x, float* y, float* theta);
void get_sensor_data(float* gyro_z, float* heading, float* vx_meas, float* vy_meas);

#endif // ROBOCUP_ODOMETRY_H
