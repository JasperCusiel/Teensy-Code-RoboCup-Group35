//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_COLOUR_SENSOR_H
#define ROBOCUP_COLOUR_SENSOR_H
#include <stdint.h>

typedef enum {COLOR_GREEN, COLOR_BLUE, COLOR_UNDEFINED} base_color_t;

typedef struct {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clear;
  base_color_t base_color;
} color_t;

bool colour_sensor_init();
void update_base_color();
base_color_t get_base_color();

#endif // ROBOCUP_COLOUR_SENSOR_H
