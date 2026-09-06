//
// Created by Jasper Cusiel on 07/08/2026.
//

// Based on https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h

#include "astar.h"
#include <math.h>

// Create node for every point in map
static astar_node_t nodes[MAP_WIDTH][MAP_HEIGHT];

static bool is_valid(int x, int y)
{
    // Check if (x,y) is in map and not occupied
    if (x < 0 || y < 0 || x >= MAP_WIDTH || y >= MAP_HEIGHT)
    {
        return false;
    }

    if (map_get_state(x, y) == OCCUPIED)
    {
        return false;
    }

    return true;
}

static uint16_t heuristic(const uint8_t x1, const uint8_t y1, const uint8_t x2, const uint8_t y2)
{
    // Heuristic is the Manhattan distance
    return abs(x1 - x2) + abs(y1 - y2);
}

// Creates the actual path from
static void make_path(uint8_t goal_x, uint8_t goal_y, path_t* path)
{
    path->length = 0;

    uint8_t x = goal_x;
    uint8_t y = goal_y;
    // Construct path in reverse (each node points to it's parent).
    while (true)
    {
        path->points[path->length].x = x;
        path->points[path->length].y = y;

        path->length++;

        // Back at start if current nodes parent points to itself (starting condition imposed in astar_find_path()).
        if (nodes[x][y].parent_x == x && nodes[x][y].parent_y == y)
        {
            break;
        }

        // Move to parent node of the current node.
        x = nodes[x][y].parent_x;
        y = nodes[x][y].parent_y;

        // Valid path can't contain more cells than exist in the entire map.
        if (path->length >= MAP_WIDTH * MAP_HEIGHT) break;
    }

    // Reverse the path so its start to goal.
    for (int i = 0; i < path->length / 2; i++)
    {
        grid_point_t temp = path->points[i];
        path->points[i] = path->points[path->length - 1 - i];
        path->points[path->length - 1 - i] = temp;
    }
}

bool astar_find_path(int start_x, int start_y, int goal_x, int goal_y, path_t* path)
{
    // Finds and constructs lowest cost path from start to goal. Returns true if valid path found, false if no valid path to goal.
    path->length = 0;

    // Check both goal and start positions are valid map positions
    if (!is_valid(start_x, start_y) || !is_valid(goal_x, goal_y))
    {
        return false;
    }

    // Initialise map nodes
    for (int8_t x = 0; x < MAP_WIDTH; x++)
    {
        for (int8_t y = 0; y < MAP_HEIGHT; y++)
        {
            nodes[x][y].x = x;
            nodes[x][y].y = y;

            nodes[x][y].g_cost = UINT16_MAX;
            nodes[x][y].h_cost = UINT16_MAX;
            nodes[x][y].f_cost = UINT16_MAX;

            nodes[x][y].parent_x = -1;
            nodes[x][y].parent_y = -1;

            nodes[x][y].opened = false;
            nodes[x][y].closed = false;
        }
    }

    // List of discovered cells that haven't been processed
    astar_node_t* open_list[MAP_WIDTH * MAP_HEIGHT];
    int open_count = 0;

    // Setup starting node
    astar_node_t* start = &nodes[start_x][start_y];

    start->g_cost = 0;
    start->h_cost = heuristic(start_x, start_y, goal_x, goal_y);
    start->f_cost = start->g_cost + start->h_cost;

    // The parent is it's self, (since we are at the start)
    start->parent_x = start_x;
    start->parent_y = start_y;
    start->opened = true;

    // Starting node must be first on open list
    open_list[open_count++] = start;

    // Continue looping until we reach the goal or there are no valid directions (no way to reach goal)
    while (open_count > 0)
    {
        // Find lowest f cost cell in open cell list
        int best = 0;
        for (int i = 1; i < open_count; i++)
        {
            if (open_list[i]->f_cost < open_list[best]->f_cost) best = i;
        }

        astar_node_t* current = open_list[best];

        // Remove from open list
        open_list[best] = open_list[--open_count];
        // Mark cell as closed so we don't back track (i.e. mark as processed).
        current->closed = true;

        // Create the path once the goal is reached.
        if (current->x == goal_x && current->y == goal_y)
        {
            make_path(goal_x, goal_y, path);
            return true;
        }

        // Check for open cells in neighboring cells
        for (int i = 0; i < 4; i++)
        {
            // Left, right, up, down
            constexpr int8_t directions[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

            // Neighbor coordinates
            uint8_t nx = current->x + directions[i][0];
            uint8_t ny = current->y + directions[i][1];

            // Check cell is in map and not occupied
            if (!is_valid(nx, ny))
                continue;

            astar_node_t* neighbor = &nodes[nx][ny];
            // Check if neighbor has already been processed
            if (neighbor->closed)
                continue;

            // One grid step has cost 1
            uint16_t new_g = current->g_cost + 1;

            // Add neighbor to processing list if it hasn't been opened or the route is shorter than the previous route.
            if (!neighbor->opened || new_g < neighbor->g_cost)
            {
                neighbor->parent_x = current->x;
                neighbor->parent_y = current->y;
                neighbor->g_cost = new_g;
                neighbor->h_cost = heuristic(nx, ny, goal_x, goal_y);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

                // Mark as opened and add to open list (new neighbor)
                if (!neighbor->opened)
                {
                    neighbor->opened = true;

                    open_list[open_count++] = neighbor;
                }
            }
        }
    }

    return false;
}
