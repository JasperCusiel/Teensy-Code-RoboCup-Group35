//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_MAPPING_H
#define ROBOCUP_MAPPING_H

// Lower-left world coordinate of the occupancy grid.
#define MAP_WORLD_MIN_X (-0.1f)
#define MAP_WORLD_MIN_Y (-0.1f)

// Generates occupancy map.
void mapping_init();

// Call periodically, gets EKF pose and inserts lidar scan into map.
void mapping_task();

// Convert world point to map coordinates.
bool world_to_map(float world_x, float world_y, int* map_x, int* map_y);

#endif // ROBOCUP_MAPPING_H
