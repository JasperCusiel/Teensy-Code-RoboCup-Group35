//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_MAPPING_H
#define ROBOCUP_MAPPING_H

#include "ToF-Sensors.h"
#include "odometry.h"

typedef struct {
  float x;
  float y;
  float theta;
}pose_t;

void mapping_init();
void mapping_task();
void update_map(pose_t pose, lidar_scan *scan);
bool world_to_map(float xw, float yw, int *mx, int *my);
void lidar_to_world(float r, float theta, const pose_t *pose, float *x, float *y);


#endif // ROBOCUP_MAPPING_H
