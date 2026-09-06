//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "occupancy-grid.h"
#include <stddef.h>
#include <math.h>

float log_odds[MAP_WIDTH][MAP_HEIGHT];

void map_init() {
  for (size_t x = 0; x < MAP_WIDTH; x++) {
    for (size_t y = 0; y < MAP_HEIGHT; y++) {
      log_odds[x][y] = 0;
    }
  }
}

void map_update_free(int x, int y) {
  // Check if in map bounds
  if(x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return;

  log_odds[x][y] = clamp_log_odds(log_odds[x][y] + MAP_LOG_ODDS_FREE);
}

void map_update_occupied(int x, int y) {
  // Check in map bounds
  if(x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return;

  log_odds[x][y] = clamp_log_odds(log_odds[x][y] + MAP_LOG_ODDS_OCC);
}

float map_get_probability(int x, int y) {
  float value = log_odds[x][y];
  return 1 / (1 + exp(-value));
}

uint8_t map_get_state(int x, int y) {
  if (log_odds[x][y] > 1.0) {
    return OCCUPIED;
  }
  if (log_odds[x][y] < -1.0) {
    return FREE;
  }
  return UNKNOWN;
}

float clamp_log_odds(float value)
{
  if (value > MAP_LOG_ODDS_MAX)
    return MAP_LOG_ODDS_MAX;

  if (value < MAP_LOG_ODDS_MIN)
    return MAP_LOG_ODDS_MIN;

  return value;
}
