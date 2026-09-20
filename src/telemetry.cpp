#include "telemetry.h"

#include <Arduino.h>
#include "ToF-Sensors.h"
#include "frontier-detection.h"
#include "lidar-config.h"
#include "mapping.h"
#include "navigation.h"
#include "odometry.h"
#include "occupancy-grid.h"

// This module is used in conjunction with 'map_viewer.py' to display the occupancy map over serial.
// Enable/disable in #include "telemetry.h"

#if MAP_TELEMETRY_ENABLED
namespace
{
    bool config_sent = false;
    uint8_t frames_since_config = 0;
    constexpr char kHexDigits[] = "0123456789ABCDEF";

    static_assert(MAP_WIDTH * MAP_HEIGHT <= 0x1000,
                  "Telemetry path encoding supports at most 4096 map cells");

    void print_float(float value) { Serial.print(value, 3); }

    void print_hex_cell_index(uint16_t index)
    {
        Serial.print(kHexDigits[(index >> 8) & 0x0f]);
        Serial.print(kHexDigits[(index >> 4) & 0x0f]);
        Serial.print(kHexDigits[index & 0x0f]);
    }

    void print_frontiers()
    {
        // FRONTIERS,<one hex digit per four row-major map cells>
        Serial.print("FRONTIERS,");
        uint8_t nibble = 0;
        uint8_t bit = 0;
        for (int index = 0; index < MAP_WIDTH * MAP_HEIGHT; ++index)
        {
            const int x = index % MAP_WIDTH;
            const int y = index / MAP_WIDTH;
            if (frontier_is_cell(x, y))
            {
                nibble |= static_cast<uint8_t>(1u << bit);
            }
            if (++bit == 4)
            {
                Serial.print(kHexDigits[nibble]);
                nibble = 0;
                bit = 0;
            }
        }
        if (bit != 0)
        {
            Serial.print(kHexDigits[nibble]);
        }
        Serial.println();
    }

    void print_path()
    {
        // PATH,<three hex digits per ordered row-major map-cell index>
        Serial.print("PATH,");
        const path_t* path = navigation_get_path();
        if (path != nullptr)
        {
            for (uint16_t i = 0; i < path->length; ++i)
            {
                const grid_point_t& point = path->points[i];
                const uint16_t index = static_cast<uint16_t>(
                    point.y * MAP_WIDTH + point.x);
                print_hex_cell_index(index);
            }
        }
        Serial.println();
    }
}

void telemetry_init()
{
    config_sent = false;
    frames_since_config = 0;
}

void telemetry_map_task()
{
    if (!config_sent || frames_since_config >= 6)
    {
        // CONFIG,width,height,cells/m,min_x,min_y,fov_min,fov_max,max_range,
        // sensor_offset_x,sensor_offset_y,num_points,free_update,occupied_update
        Serial.print("CONFIG,");
        Serial.print(MAP_WIDTH);
        Serial.print(',');
        Serial.print(MAP_HEIGHT);
        Serial.print(',');
        print_float(MAP_CELLS_PER_M);
        Serial.print(',');
        print_float(MAP_WORLD_MIN_X);
        Serial.print(',');
        print_float(MAP_WORLD_MIN_Y);
        Serial.print(',');
        print_float(FOV_MIN);
        Serial.print(',');
        print_float(FOV_MAX);
        Serial.print(',');
        print_float(MAX_TOF_RANGE);
        Serial.print(',');
        print_float(TOF_ARRAY_OFFSET_X);
        Serial.print(',');
        print_float(TOF_ARRAY_OFFSET_Y);
        Serial.print(',');
        Serial.print(NUM_POINTS);
        Serial.print(',');
        print_float(MAP_LOG_ODDS_FREE);
        Serial.print(',');
        print_float(MAP_LOG_ODDS_OCC);
        Serial.println();
        config_sent = true;
        frames_since_config = 0;
    }

    pose_t pose;
    get_ekf_pose(&pose.x, &pose.y, &pose.theta);
    const lidar_scan* scan = get_scan();
    // MAP,x,y,theta,range_0,...,range_(NUM_POINTS-1)
    Serial.print("MAP,");
    print_float(pose.x);
    Serial.print(',');
    print_float(pose.y);
    Serial.print(',');
    print_float(pose.theta);
    for (int i = 0; i < NUM_POINTS; ++i)
    {
        Serial.print(',');
        print_float(scan->ranges[i]);
    }
    Serial.println();

    print_frontiers();
    print_path();
    ++frames_since_config;
}
#else
void telemetry_init()
{
}
void telemetry_map_task()
{
}
#endif
