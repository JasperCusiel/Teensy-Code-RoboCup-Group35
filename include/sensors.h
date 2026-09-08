//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_SENSORS_H
#define ROBOCUP_SENSORS_H

// Represents each sensor to start.
typedef struct
{
    bool boot_okay;
    bool (*init_func)();
    const char* name;
} sensor_t;

#define NUM_SENSORS 10

// Starts all the sensors.
void sensors_init();

// Getter functions used by display to show which sensor failed to start (if any).
bool sensors_boot_okay();
const char* sensors_failed_sensor();

#endif // ROBOCUP_SENSORS_H
