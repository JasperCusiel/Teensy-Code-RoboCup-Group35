//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "ray-trace.h"
#include <math.h>
#include "occupancy-grid.h"

// This module implements ray casting based on Bresenham's line algorthm.
// From: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm


void ray_cast(const int x0, const int y0, const int x1, const int y1, const ray_callback_t fill_callback,
              const ray_callback_t end_point_callback)
{
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

        // Don't access outside map
        if (x < 0 || x >= MAP_WIDTH ||
            y < 0 || y >= MAP_HEIGHT)
            break;

        // Mark the endpoint
        if (fill_callback != nullptr)
        {
            fill_callback(x, y);
        }

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    // Only mark endpoint if it is inside the map
    if (x1 >= 0 && x1 < MAP_WIDTH &&
        y1 >= 0 && y1 < MAP_HEIGHT)
    {
        if (end_point_callback != nullptr)
        {
            end_point_callback(x1, y1);
        }
    }
}
