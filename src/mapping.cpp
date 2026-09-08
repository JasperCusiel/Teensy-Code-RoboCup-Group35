//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "mapping.h"

#include "occupancy-grid.h"
#include "ray-trace.h"
#include <math.h>
#include "odometry.h"
#include "ToF-Sensors.h"


namespace
{
    void lidar_to_robot(float r, float theta, float* xr, float* yr)
    {
        // Function converts lidar frame to robot frame with right hand coordinate system (CCW +ve)
        *xr = -r * sinf(theta);
        *yr = r * cosf(theta);

        // Add sensor offset
        *xr += TOF_ARRAY_OFFSET_X;
        *yr += TOF_ARRAY_OFFSET_Y;
    }

    void lidar_to_world(float r, float theta, const pose_t* pose, float* x, float* y)
    {
        // Function converts lidar polar values into world coordinates.
        float robot_x, robot_y;

        // Convert lidar to robot frame
        lidar_to_robot(r, theta, &robot_x, &robot_y);

        // Rotate into world frame
        float xw = robot_x * cosf(pose->theta) - robot_y * sinf(pose->theta);
        float yw = robot_x * sinf(pose->theta) + robot_y * cosf(pose->theta);

        // Translate to world frame
        xw += pose->x;
        yw += pose->y;

        // Pass out of func
        *x = xw;
        *y = yw;
    }

    void update_map(pose_t pose, lidar_scan* scan)
    {
        // Convert robot position to map coordinates
        int robot_x, robot_y;

        if (!world_to_map(pose.x, pose.y, &robot_x, &robot_y))
        {
            return; // Robot position to map coordinates failed
        }

        // Insert lidar scan points one at a time.
        for (int i = 0; i < NUM_POINTS; i++)
        {
            float r = scan->ranges[i];
            float theta = scan->angles[i];

            // Check values are valid
            if (!isfinite(r) || !isfinite(theta) || r <= 0.0f)
            {
                continue;
            }

            // Ignore values greater than the max sensing range.
            bool obstacle_detected = r < MAX_TOF_RANGE;

            // Limit maximum sensing distance
            if (r > MAX_TOF_RANGE)
            {
                r = MAX_TOF_RANGE;
            }
            float world_x, world_y; // World X, Y
            int map_x, map_y; // Map X, Y

            // Convert lidar end point to (x,y) in world frame.
            lidar_to_world(r, theta, &pose, &world_x, &world_y);

            if (!world_to_map(world_x, world_y, &map_x, &map_y))
            {
                continue; // Skip inserting ray if not valid
            }

            // Mark endpoint as occupied or free based on lidar range reading.
            if (obstacle_detected)
            {
                ray_cast(robot_x, robot_y, map_x, map_y, map_update_free, map_update_occupied);
            }
            else
            {
                ray_cast(robot_x, robot_y, map_x, map_y, map_update_free, nullptr);
            }
        }
    }
}

bool world_to_map(float world_x, float world_y, int* map_x, int* map_y)
{
    // Function converts world coordinates to map coordinates, returns true if position inside map, false if not.
    // Origin bottom left
    /* Y
     * ^
     * |
     * |---> X */
    *map_x = (int)floorf((world_x - MAP_WORLD_MIN_X) * MAP_CELLS_PER_M);
    *map_y = (int)floorf((world_y - MAP_WORLD_MIN_Y) * MAP_CELLS_PER_M);

    // Check it's within bounds of the map
    if (*map_x < 0 || *map_x > (MAP_WIDTH - 1) || *map_y < 0 || *map_y > (MAP_HEIGHT - 1))
    {
        // Position outside of map coordinates
        return false;
    }

    return true;
}

void mapping_init()
{
    // Generate the occupancy grid.
    map_init();
}

void mapping_task()
{
    // Get latest pose from EKF
    pose_t current_pose;
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);

    // Get a fresh lidar scan
    lidar_scan* current_scan = get_scan();

    // Put this new scan into the map.
    update_map(current_pose, current_scan);
}
