//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_IMU_H
#define ROBOCUP_IMU_H

#include <stdbool.h>
bool imu_init();
void displaySensorDetails(void);
void displaySensorStatus(void);

struct imu_data {
  float heading;
  float gyro_z;
  float accel_x;
  float accel_y;
};
void imu_get_reading();
float imu_get_heading();
float imu_get_gyro_z();
#endif // ROBOCUP_IMU_H
