//
// Created by Jasper Cusiel on 25/07/2026.
//

#ifndef ROBOCUP_VFH_H
#define ROBOCUP_VFH_H

#include <lidar-config.h>

#define MAX_RANGE 1.3f
#define VFH_BLOCKED_THRESHOLD 0.95f
#define VFH_FREE_THRESHOLD 0.70f
#define THRESHOLD VFH_BLOCKED_THRESHOLD
#define ROBOT_RADIUS 0.24f
#define SAFETY_DIST 0.02f
#define ROBOT_CLEARANCE (ROBOT_RADIUS + SAFETY_DIST)
#define VFH_INFLATION_SCALE 0.55f
#define FRONT_CLEARANCE_CONE radians(10.0f)

struct VFH
{
    float histogram[NUM_SECTORS];
    bool free_directions[NUM_SECTORS];
    float sector_angles[NUM_SECTORS];
    float target_angle;
    float steering_angle;
    float forward_clearance;
};


void vfh_init();
void add_histogram_value(float vfh_histogram[NUM_SECTORS], int sector,
                         float weight, float range);
void build_histogram();
void threshold_histogram();
float vfh_get_best_direction(float target_angle);
void compute_vfh();
float* vfh_get_histogram();
void vfh_set_target_angle(float target_angle);
float vfh_get_target_angle();
float vfh_get_steering_angle();
float vfh_get_forward_clearance();

#endif // ROBOCUP_VFH_H
