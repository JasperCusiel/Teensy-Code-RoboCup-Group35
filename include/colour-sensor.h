//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_COLOUR_SENSOR_H
#define ROBOCUP_COLOUR_SENSOR_H
#include <stdint.h>

typedef enum { COLOR_GREEN, COLOR_BLUE, COLOR_UNDEFINED } base_color_t;

// Used to store base colour profile
typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
    base_color_t base_color;
} color_t;

// Starts sensor and sets the base color.
bool colour_sensor_init();

// Gets raw data and determines base color
void update_base_color();

// Returns stored base color
base_color_t get_base_color();

#endif // ROBOCUP_COLOUR_SENSOR_H
