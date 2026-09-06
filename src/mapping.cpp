//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "mapping.h"
#include "occupancy-grid.h"
#include "ray-trace.h"
#include <Arduino.h>
#include <math.h>


void mapping_init() {
  map_init();
}

void mapping_task() {
  pose_t current_pose;
  get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
  // Serial.printf("POSE: x=%.4f y=%.4f theta=%.4f\n",
                // current_pose.x,
                // current_pose.y,
                // current_pose.theta);

  lidar_scan *current_scan = get_scan();
  update_map(current_pose, current_scan);
}

void update_map(pose_t pose, lidar_scan *scan) {
  // convert robot position to map coordinates
  int rx, ry;
  if (!world_to_map(pose.x, pose.y, &rx, &ry)) {
    // Serial.println("robot position to map coordinates failed");
    return;
  }
  // Serial.printf("rx: %d, ry: %d", rx, ry);

  for(int i=0;i<NUM_POINTS;i++)
  {
    float r = scan->ranges[i];

    float theta = scan->angles[i];

    if (!isfinite(r) || !isfinite(theta) || r <= 0.0f) {
      continue;
    }

    bool obstacle_detected = r < MAX_TOF_RANGE;

    // Limit maximum sensing distance
    if (r > MAX_TOF_RANGE) {
      r = MAX_TOF_RANGE;
    }

    // Serial.printf("i=%d r=%.3f theta=%.3f\n", i, r, theta);

    float wx, wy; // World X, Y
    int mx, my; // Map X, Y
    lidar_to_world(r, theta, &pose, &wx, &wy);



    if (!world_to_map(wx,wy, &mx, &my)) {
      // Serial.println("inserting ray failed");
      continue; // Skip inserting ray if not valid
    }
    if (obstacle_detected) {
      ray_cast(rx, ry, mx, my, map_update_free, map_update_occupied);
    } else {
      ray_cast(rx, ry, mx, my, map_update_free, nullptr);
    }
  }
}

void lidar_to_robot(float r, float theta, float *xr, float *yr) {
  // theta is a signed bearing from forward (+Y), CCW-positive. A point
  // straight ahead therefore becomes (0, r) in the robot frame.
  *xr = -r * sinf(theta);
  *yr =  r * cosf(theta);

  // Add sensor offset
  *xr += TOF_ARRAY_OFFSET_X;
  *yr += TOF_ARRAY_OFFSET_Y;
}

bool world_to_map(float xw, float yw, int *mx, int *my) {
  // Convert world coordinates to map coordinates
  // Origin bottom left
  /* Y
   * ^
   * |
   * |---> X */
  // Serial.printf("xw: %f, yw: %f\n", xw, yw);
  *mx = (int)floorf((xw - MAP_WORLD_MIN_X) * MAP_CELLS_PER_M);
  *my = (int)floorf((yw - MAP_WORLD_MIN_Y) * MAP_CELLS_PER_M);

  // Check it's within bounds of the map
  if (*mx < 0 || *mx > (MAP_WIDTH - 1)||
        *my < 0 || *my > (MAP_HEIGHT - 1))
  {
    // Serial.println("Position out of map coordinates");
    // Serial.printf("mx: %d, my: %d\n", *mx, *my);
    return false;
  }

  return true;

}

void lidar_to_world(float r, float theta, const pose_t *pose, float *x, float *y) {

  float xr,yr;
  lidar_to_robot(r, theta, &xr, &yr); // Lidar to robot

  // Rotate into world
  float xw = xr * cosf(pose->theta) - yr * sinf(pose->theta);
  float yw = xr * sinf(pose->theta) + yr * cosf(pose->theta);

  // Translate to world
  xw += pose->x;
  yw += pose->y;

  // Pass out of func
  *x = xw;
  *y = yw;
}
