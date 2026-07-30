//
// Created by Jasper Cusiel on 25/07/2026.
//

#ifndef ROBOCUP_VFH_H
#define ROBOCUP_VFH_H

#include <lidar-config.h>
struct VFH {
  float histogram[NUM_SECTORS];
  bool free_directions[NUM_SECTORS];
  float sector_angles[NUM_SECTORS];
  float target_angle;
  float steering_angle;
};


void vfh_init();
void add_histogram_value(float vfh_histogram[NUM_SECTORS], int sector,
                         float weight, float range);
void build_histogram();
void threshold_histogram();
float get_best_direction(float target_angle);
void compute_vfh();
float* get_histogram();
void set_target_angle(const float target_angle);
float get_target_angle();
float get_steering_angle();

#endif // ROBOCUP_VFH_H
