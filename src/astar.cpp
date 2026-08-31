//
// Created by Jasper Cusiel on 07/08/2026.
//

// Based on https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h

#include "astar.h"
#include <math.h>
#include <cfloat>

#define X_STEP 1
#define Y_STEP 1

static astar_node_t nodes[MAP_WIDTH][MAP_HEIGHT];

static bool is_valid(int x, int y) {
  if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT) {
    return false;
  }

  if (map_get_state(x, y) == OCCUPIED) {
    return false;
  }

  return true;
}

static float heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

static void make_path(int goal_x,
                      int goal_y, path_t *path) {
  path->length = 0;

  int x = goal_x;
  int y = goal_y;

  while (true) {
    path->points[path->length].x = x;
    path->points[path->length].y = y;

    path->length++;

    if (nodes[x][y].parent_x == x && nodes[x][y].parent_y == y) {
      break;
    }

    int px = nodes[x][y].parent_x;
    int py = nodes[x][y].parent_y;

    x = px;
    y = py;

    if (path->length >= MAP_WIDTH * MAP_HEIGHT)
      break;
  }

  // Reverse path
  for (int i = 0; i < path->length / 2; i++) {
    grid_point_t temp = path->points[i];

    path->points[i] = path->points[path->length - 1 - i];

    path->points[path->length - 1 - i] = temp;
  }
}

bool astar_find_path(int start_x, int start_y, int goal_x, int goal_y,
                     path_t *path) {

  path->length = 0;

  if (!is_valid(start_x, start_y) || !is_valid(goal_x, goal_y)) {
    return false;
  }

  // Initialise nodes

  for (int x = 0; x < MAP_WIDTH; x++) {
    for (int y = 0; y < MAP_HEIGHT; y++) {
      nodes[x][y].x = x;
      nodes[x][y].y = y;

      nodes[x][y].g_cost = FLT_MAX;
      nodes[x][y].h_cost = FLT_MAX;
      nodes[x][y].f_cost = FLT_MAX;

      nodes[x][y].parent_x = -1;
      nodes[x][y].parent_y = -1;

      nodes[x][y].opened = false;
      nodes[x][y].closed = false;
    }
  }

  astar_node_t *open_list[MAP_WIDTH * MAP_HEIGHT];
  int open_count = 0;

  // Start node

  astar_node_t *start = &nodes[start_x][start_y];

  start->g_cost = 0;
  start->h_cost = heuristic(start_x, start_y, goal_x, goal_y);

  start->f_cost = start->g_cost + start->h_cost;

  start->parent_x = start_x;
  start->parent_y = start_y;

  start->opened = true;

  open_list[open_count++] = start;

  int directions[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (open_count > 0) {

    // Find lowest f cost

    int best = 0;

    for (int i = 1; i < open_count; i++) {
      if (open_list[i]->f_cost < open_list[best]->f_cost) {
        best = i;
      }
    }

    astar_node_t *current = open_list[best];

    // Remove from open list

    open_list[best] = open_list[--open_count];

    current->closed = true;

    // Goal reached

    if (current->x == goal_x && current->y == goal_y) {
      make_path(goal_x, goal_y, path);

      return true;
    }

    // Check neighbors

    for (int i = 0; i < 4; i++) {
      int nx = current->x + directions[i][0];

      int ny = current->y + directions[i][1];

      if (!is_valid(nx, ny))
        continue;

      astar_node_t *neighbor = &nodes[nx][ny];

      if (neighbor->closed)
        continue;

      float new_g = current->g_cost + 1.0f;

      if (!neighbor->opened || new_g < neighbor->g_cost) {

        neighbor->parent_x = current->x;
        neighbor->parent_y = current->y;
        neighbor->g_cost = new_g;
        neighbor->h_cost = heuristic(nx, ny, goal_x, goal_y);
        neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

        if (!neighbor->opened) {
          neighbor->opened = true;

          open_list[open_count++] = neighbor;
        }
      }
    }
  }

  return false;
}