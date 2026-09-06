//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_OCCUPANCY_GRID_H
#define ROBOCUP_OCCUPANCY_GRID_H

#include <stdint.h>

#define MAP_CELLS_PER_M 10.0f
#define MAP_M_PER_CELL (float)(1.0f /MAP_CELLS_PER_M)

// Shared mapping evidence values. Telemetry sends these so the desktop viewer
// reconstructs the same occupancy grid as the firmware.
#define MAP_LOG_ODDS_MIN (-5.0f)
#define MAP_LOG_ODDS_MAX  5.0f
#define MAP_LOG_ODDS_FREE (-0.4f)
#define MAP_LOG_ODDS_OCC   0.85f

#define MAP_WIDTH (int8_t)(3.0f * MAP_CELLS_PER_M)
#define MAP_HEIGHT (int8_t)(5.0f * MAP_CELLS_PER_M)


enum occupancy_state_t
{
    UNKNOWN,
    FREE,
    OCCUPIED
};

void map_init();
void map_update_free(int x, int y);
void map_update_occupied(int x, int y);
float map_get_probability(int x, int y);
uint8_t map_get_state(int x, int y);
float clamp_log_odds(float value);

#endif // ROBOCUP_OCCUPANCY_GRID_H
