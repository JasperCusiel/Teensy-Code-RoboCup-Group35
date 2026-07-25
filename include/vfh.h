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
};

void vfh_init(VFH *vfh);
void add_histogram_value(float vfh_histogram[NUM_SECTORS], int sector,
                         float weight, float range);
void build_histogram(VFH *vfh, const lidar_scan *new_lidar_scan);
void threshold_histogram(VFH *vfh);
float get_best_direction(VFH *vfh, float target_angle);
float compute_vfh(VFH *vfh, const lidar_scan *scan, float *target_angle);

#endif // ROBOCUP_VFH_H
