//
// Created by Jasper Cusiel on 07/08/2026.
//

#ifndef ROBOCUP_FRONTIER_DETECTION_H
#define ROBOCUP_FRONTIER_DETECTION_H

#include <stdint.h>

// A navigable target on the boundary between mapped free space and unmapped space.
typedef struct
{
    int x;
    int y;
    uint16_t size;
} frontier_goal_t;

// Returns true when this free cell borders at least one unknown cell.
bool frontier_is_cell(int x, int y);

// Searches the occupancy for 8-connected frontier clusters.
// Selects the largest cluster and returns the frontier cell closest to the centroid of the frontier cluster.
// Returns the coordinates of the largest frontier or closest if there are equal size largest frontiers.

// Returns true if frontier found and populates goal, false if not.
bool frontier_find_largest_goal(int robot_x, int robot_y,
                                frontier_goal_t* goal);

#endif // ROBOCUP_FRONTIER_DETECTION_H
