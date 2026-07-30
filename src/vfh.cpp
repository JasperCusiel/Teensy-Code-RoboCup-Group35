//
// Created by Jasper Cusiel on 25/07/2026.
//

#include <ToF-Sensors.h>
#include <math.h>
#include <stdint.h>
#include <vfh.h>

#define MAX_RANGE 1.3f
#define THRESHOLD 2.0f
#define ROBOT_RADIUS 0.2f
#define SAFETY_DIST 0.05f
#define ROBOT_CLEARANCE (ROBOT_RADIUS + SAFETY_DIST)

VFH vfh;

void vfh_init() {
  for (int i = 0; i < NUM_SECTORS; i++) {
    vfh.sector_angles[i] = FOV_MIN + (i + 0.5f) * SECTOR_WIDTH;
  }
}

void add_histogram_value(float vfh_histogram[NUM_SECTORS], int sector,
                         float weight, float range) {
  int spread = (int)ceilf(atan2f(ROBOT_CLEARANCE, range) / SECTOR_WIDTH);
  spread = 1;
  for (int i = -spread; i <= spread; i++) {
    int s = sector + i;

    if (s >= 0 && s < NUM_SECTORS) {
      vfh_histogram[s] += weight;
    }
  }
}

void build_histogram() {
  lidar_scan* new_lidar_scan = get_scan();
  for (size_t i = 0; i < NUM_SECTORS; i++) {
    vfh.histogram[i] = 0.0f;
  }

  for (size_t i = 0; i < NumOfTOFSensors * NumOfZonesPerSensor; i++) {
    float r = new_lidar_scan->ranges[i];

    if (r <= 0.01 || r > MAX_RANGE) {
      continue;
    }
    // closer obstacles = higher density
    float x = (MAX_RANGE - r) / MAX_RANGE;
    float weight = x * x;

    uint8_t sector = new_lidar_scan->sector_index[i];

    if (sector < NUM_SECTORS) {
      add_histogram_value(vfh.histogram, sector, weight, r);
    }
  }
}

void threshold_histogram() {
  for (int i = 0; i < NUM_SECTORS; i++) {
    vfh.free_directions[i] = (vfh.histogram[i] < THRESHOLD);
  }
}

float get_best_direction(float target_angle) {
  float best_angle = NAN;
  float best_cost = 1e9;

  for (size_t i = 0; i < NUM_SECTORS; i++) {
    if (!vfh.free_directions[i]) {
      continue;
    }
    // sector to angle
    float angle = vfh.sector_angles[i];

    float diff = fabs(angle - target_angle);
    float cost = diff + vfh.histogram[i] * 0.5f;

    if (cost < best_cost) {
      best_cost = cost;
      best_angle = angle;
    }
  }

  return best_angle;
}

void compute_vfh() {
  build_histogram();
  threshold_histogram();
  vfh.steering_angle = get_best_direction(vfh.target_angle);
}

float* get_histogram() {
  return vfh.histogram;
}

void set_target_angle(const float target_angle) {
  vfh.target_angle = target_angle;
}

float get_target_angle() {
  return vfh.target_angle;
}

float get_steering_angle() {
  return vfh.steering_angle;
}