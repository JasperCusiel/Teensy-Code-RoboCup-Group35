//
// Created by Jasper Cusiel on 07/08/2026.
//

#ifndef ROBOCUP_ASTAR_H
#define ROBOCUP_ASTAR_H
#include "occupancy-grid.h"
#include <stdint.h>

typedef struct {
  int x;
  int y;

} grid_point_t;

typedef struct {
  grid_point_t points[MAP_WIDTH * MAP_HEIGHT];
  uint16_t length;

} path_t;

typedef struct {
  int8_t x;
  int8_t y;

  int8_t parent_x;
  int8_t parent_y;

  float g_cost;
  float h_cost;
  float f_cost;

  bool opened;
  bool closed;
} astar_node_t;

inline bool operator<(const astar_node_t &lhs,
                      const astar_node_t &rhs) { // We need to overload "<" to
                                                 // put our struct into a set
  return lhs.f_cost < rhs.f_cost;
}

void astar_init();
bool astar_find_path(int start_x, int start_y, int goal_x, int goal_y, path_t *path);

#endif // ROBOCUP_ASTAR_H
