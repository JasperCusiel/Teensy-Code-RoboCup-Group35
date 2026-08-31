//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "mapping.h"
#include "occupancy-grid.h"
#include "ray-trace.h"
#include <Arduino.h>
#include <math.h>


float origin_x = -1.5f;  // [m]
float origin_y = -1.0f; // [m]

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
    Serial.println("robot position to map coordinates failed");
    return;
  }
  Serial.printf("rx: %d, ry: %d", rx, ry);

  for(int i=0;i<NUM_POINTS;i++)
  {
    float r = scan->ranges[i];

    float theta = scan->angles[i];

    // Serial.printf("i=%d r=%.3f theta=%.3f\n", i, r, theta);

    float wx, wy; // World X, Y
    int mx, my; // Map X, Y
    lidar_to_world(r, theta, &pose, &wx, &wy);
    Serial.printf("wx: %f, wy: %f\n", wx, wy);


    if (!world_to_map(wx,wy, &mx, &my)) {
      continue; // Skip inserting ray if not valid
    }
    if (scan->ranges[i] >= MAX_TOF_RANGE) {
      ray_cast(rx, ry, mx, my, map_update_free, nullptr);
    } else {
      ray_cast(rx, ry, mx, my, map_update_free, map_update_occupied);
    }
  }
}

void lidar_to_robot(float r, float theta, float *xr, float *yr) {
  // Convert polar to cartesian in robot frame
  *xr = r * cos(theta);
  *yr = r * sin(theta);

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

  *mx = (int)floorf((xw - origin_x) / MAP_M_PER_CELL);
  *my = (int)floorf((yw - origin_y) / MAP_M_PER_CELL);

  // Check it's within bounds of the map
  if (*mx < 0 || *mx > (MAP_WIDTH - 1)||
        *my < 0 || *my > (MAP_HEIGHT - 1))
  {
    Serial.println("Position out of map coordinates");
    Serial.printf("mx: %d, my: %d\n", *mx, *my);
    return false;
  }

  return true;

}

void lidar_to_world(float r, float theta, const pose_t *pose, float *x, float *y) {

  float xr,yr;
  lidar_to_robot(r, theta, &xr, &yr);

  // Rotate into world
  float xw = xr * cos(pose->theta) - yr * sin(pose->theta);
  float yw = xr * sin(pose->theta) + yr * cos(pose->theta);

  // Translate to world
  xw += pose->x;
  yw += pose->y;

  // Pass out of func
  *x = xw;
  *y = yw;
}
