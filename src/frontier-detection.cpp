//
// Created by Jasper Cusiel on 07/08/2026.
//
#include "frontier-detection.h"
#include "occupancy-grid.h"
#include <limits.h>

// Frontier base exploration algorithm. A frontier is a known free map cell next to unknown space.
// The robot selects the largest connected frontier and navigates towards its center

namespace
{
    // Grid coordinate used internally while traversing frontiers
    struct cell_t
    {
        int x;
        int y;
    };

    // Number of cells in map
    constexpr int kMapCellCount = MAP_WIDTH * MAP_HEIGHT;

    // Checks if given (x,y) is valid map cell
    bool in_map(int x, int y)
    {
        return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
    }

    // Squared Euclidean distance is used for comparisons as it avoids square roots.
    int distance_squared(int x0, int y0, int x1, int y1)
    {
        const int dx = x0 - x1;
        const int dy = y0 - y1;
        return dx * dx + dy * dy;
    }
} // namespace

bool frontier_is_cell(int x, int y)
{
    // Function determines if the given cell is frontier: a frontier cell is a known free cell with at least one unknown neighbor.

    // Check if cell is both in map and free.
    if (!in_map(x, y))
    {
        return false;
    }
    if (map_get_state(x, y) != FREE)
    {
        return false;
    }

    // Check the 8 cells around the target cell (diagonally touching unknown cells count)
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            const int nx = x + dx;
            const int ny = y + dy;

            // Frontier found as soon as there is one unknown cell touching the target cell
            if (in_map(nx, ny) && map_get_state(nx, ny) == UNKNOWN)
            {
                return true;
            }
        }
    }
    return false;
}

bool frontier_find_largest_goal(int robot_x, int robot_y, frontier_goal_t* goal)
{
    // Function takes in the robots position on the map and outputs goal frontier.
    // Returns true if frontier was found and goal populated, false if not.
    if (goal == nullptr)
    {
        return false;
    }

    bool visited[MAP_WIDTH][MAP_HEIGHT] = {}; // To keep track of frontier cells that have been processed.
    cell_t stack[kMapCellCount]; // Used fpr non-recursive depth first flood fill.
    cell_t component[kMapCellCount]; // Used to store every cell belonging to the current frontier cluster.

    bool found = false;
    uint16_t largest_size = 0;
    int best_robot_distance = INT_MAX;

    // Scan map for unvisited frontier cell to use as start of new cluster
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
        for (int y = 0; y < MAP_HEIGHT; ++y)
        {
            if (visited[x][y] || !frontier_is_cell(x, y))
            {
                continue;
            }

            // Flood-fill one 8-connected frontier cluster.
            int stack_size = 0;
            int component_size = 0;
            int sum_x = 0;
            int sum_y = 0;

            // Add start cell to stack.
            stack[stack_size++] = {x, y};
            // Mark as visited so it can't be pushed twice.
            visited[x][y] = true;

            while (stack_size > 0)
            {
                const cell_t current = stack[--stack_size];
                component[component_size++] = current;

                // Sum coordinates to calculate centroid after
                sum_x += current.x;
                sum_y += current.y;

                // Examine 8 neighboring cells
                for (int dx = -1; dx <= 1; ++dx)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        if (dx == 0 && dy == 0)
                        {
                            continue;
                        }

                        const int nx = current.x + dx;
                        const int ny = current.y + dy;
                        if (!in_map(nx, ny) || visited[nx][ny] ||
                            !frontier_is_cell(nx, ny))
                        {
                            continue;
                        }
                        // Store if in map, not visited and a frontier cell
                        visited[nx][ny] = true;
                        stack[stack_size++] = {nx, ny};
                    }
                }
            }
            // Calculate the centroid of the frontier cluster
            const int centroid_x = sum_x / component_size;
            const int centroid_y = sum_y / component_size;

            // Find the largest frontier, as it represents the largest boundary of unexplored space.
            // If equal, prefer the closer frontier.
            int target_index = 0;
            int target_centroid_distance = INT_MAX;
            for (int i = 0; i < component_size; ++i)
            {
                const int centroid_distance = distance_squared(
                    component[i].x, component[i].y, centroid_x, centroid_y);
                if (centroid_distance < target_centroid_distance)
                {
                    target_centroid_distance = centroid_distance;
                    target_index = i;
                }
            }

            const cell_t target = component[target_index];
            const int robot_distance = distance_squared(target.x, target.y,
                                                        robot_x, robot_y);
            if (!found || component_size > largest_size ||
                (component_size == largest_size &&
                    robot_distance < best_robot_distance))
            {
                found = true;
                largest_size = static_cast<uint16_t>(component_size);
                best_robot_distance = robot_distance;
                goal->x = target.x; // Frontier centroid X coordinate
                goal->y = target.y; // Frontier centroid Y coordinate
                goal->size = largest_size; // Number of cells in frontier cluster
            }
        }
    }

    return found;
}
