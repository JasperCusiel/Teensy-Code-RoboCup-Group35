//
// Created by Jasper Cusiel on 28/07/2026.
//

#ifndef ROBOCUP_ODOMETRY_H
#define ROBOCUP_ODOMETRY_H


void odometry_init();
void odometry_update(float dt);
void get_ekf_pose(float *out);

#endif // ROBOCUP_ODOMETRY_H
