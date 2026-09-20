//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_OPTICAL_FLOW_H
#define ROBOCUP_OPTICAL_FLOW_H

// Start sensor.
bool optical_flow_init();

// Gets x and y velocity base on dt since last call. (optical flow accumulates between calls so dt is needed to convert to velocity).
void flow_get_velocity(float* vx, float* vy, float dt);

// Compensates x,y velocities for sensor offset using gyro data.
void compensate_flow(float* vx, float* vy, float omega);

#endif // ROBOCUP_OPTICAL_FLOW_H
