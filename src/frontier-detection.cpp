//
// Created by Jasper Cusiel on 07/08/2026.
//
#include "frontier-detection.h"
#include "occupancy-grid.h"

#include <limits.h>
// Finds largest unexplored frontier and set as goal
namespace {

struct cell_t {
  int x;
  int y;
};

constexpr int kMapCellCount = MAP_WIDTH * MAP_HEIGHT;

bool in_map(int x, int y) {
  return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

} // namespace

bool frontier_is_cell(int x, int y) {
  if (!in_map(x, y)) {
    return false;
  }
  if (map_get_state(x, y) != FREE) {
    return false;
  }

  // Match the MATLAB implementation's 8-neighbour definition.
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      const int nx = x + dx;
      const int ny = y + dy;
      if (in_map(nx, ny) && map_get_state(nx, ny) == UNKNOWN) {
        return true;
      }
    }
  }
  return false;
}

namespace {

int distance_squared(int x0, int y0, int x1, int y1) {
  const int dx = x0 - x1;
  const int dy = y0 - y1;
  return dx * dx + dy * dy;
}

} // namespace

bool frontier_find_largest_goal(int robot_x, int robot_y,
                                frontier_goal_t *goal) {
  if (goal == nullptr) {
    return false;
  }

  bool visited[MAP_WIDTH][MAP_HEIGHT] = {};
  cell_t stack[kMapCellCount];
  cell_t component[kMapCellCount];

  bool found = false;
  uint16_t largest_size = 0;
  int best_robot_distance = INT_MAX;

  for (int x = 0; x < MAP_WIDTH; ++x) {
    for (int y = 0; y < MAP_HEIGHT; ++y) {
      if (visited[x][y] || !frontier_is_cell(x, y)) {
        continue;
      }

      // Flood-fill one 8-connected frontier cluster.
      int stack_size = 0;
      int component_size = 0;
      int sum_x = 0;
      int sum_y = 0;
      stack[stack_size++] = {x, y};
      visited[x][y] = true;

      while (stack_size > 0) {
        const cell_t current = stack[--stack_size];
        component[component_size++] = current;
        sum_x += current.x;
        sum_y += current.y;

        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
              continue;
            }

            const int nx = current.x + dx;
            const int ny = current.y + dy;
            if (!in_map(nx, ny) || visited[nx][ny] ||
                !frontier_is_cell(nx, ny)) {
              continue;
            }

            visited[nx][ny] = true;
            stack[stack_size++] = {nx, ny};
          }
        }
      }

      const int centroid_x = sum_x / component_size;
      const int centroid_y = sum_y / component_size;

      int target_index = 0;
      int target_centroid_distance = INT_MAX;
      for (int i = 0; i < component_size; ++i) {
        const int centroid_distance = distance_squared(
            component[i].x, component[i].y, centroid_x, centroid_y);
        if (centroid_distance < target_centroid_distance) {
          target_centroid_distance = centroid_distance;
          target_index = i;
        }
      }

      const cell_t target = component[target_index];
      const int robot_distance = distance_squared(target.x, target.y,
                                                  robot_x, robot_y);
      if (!found || component_size > largest_size ||
          (component_size == largest_size &&
           robot_distance < best_robot_distance)) {
        found = true;
        largest_size = static_cast<uint16_t>(component_size);
        best_robot_distance = robot_distance;
        goal->x = target.x;
        goal->y = target.y;
        goal->size = largest_size;
      }
    }
  }

  return found;
}
