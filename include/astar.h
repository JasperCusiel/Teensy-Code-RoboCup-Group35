//
// Created by Jasper Cusiel on 07/08/2026.
//

#ifndef ROBOCUP_ASTAR_H
#define ROBOCUP_ASTAR_H
#include "occupancy-grid.h"
#include <stdint.h>

typedef struct
{
    int x;
    int y;
} grid_point_t;

typedef struct
{
    grid_point_t points[MAP_WIDTH * MAP_HEIGHT];
    uint16_t length;
} path_t;

typedef struct
{
    // New cell to check
    int8_t x;
    int8_t y;

    // Previous cell on the best route
    int8_t parent_x;
    int8_t parent_y;

    uint16_t g_cost; // Exact distance from start
    uint16_t h_cost; // Estimated distance to the goal
    uint16_t f_cost; // g_cost + h_cost

    bool opened; // Discovered node
    bool closed; // Already processed node
} astar_node_t;

void astar_init();
bool astar_find_path(int start_x, int start_y, int goal_x, int goal_y, path_t* path);

#endif // ROBOCUP_ASTAR_H
