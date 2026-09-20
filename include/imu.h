//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_IMU_H
#define ROBOCUP_IMU_H

// Used to store the imu data
struct imu_data
{
    float heading;
    float gyro_z;
    float accel_x;
    float accel_y;
};

// Starts imu and ensure fusion starts and no errors occur, returns true if start okay, false if not.
bool imu_init();

// From Adafruit library documentation, used for debugging.
void displaySensorDetails();
void displaySensorStatus();

// Task to sample the imu for new data
void imu_task();

// Getter functions to expose imu data
float imu_get_heading();
float imu_get_gyro_z();

#endif // ROBOCUP_IMU_H
