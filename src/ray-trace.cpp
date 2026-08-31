//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "ray-trace.h"
#include <math.h>

// From: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
void ray_cast(const int x0, const int y0, const int x1, const int y1, const ray_callback_t fill_callback, const ray_callback_t end_point_callback) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);

  int sx = (x1 > x0) ? 1 : -1;
  int sy = (y1 > y0) ? 1 : -1;

  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true)
  {
    // Stop before end point
    if (x == x1 && y == y1)
      break;

    fill_callback(x, y);  // mark free space

    int e2 = 2 * err;

    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }

    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  // Mark endpoint
  end_point_callback(x1, y1);
}