//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_SENSORS_H
#define ROBOCUP_SENSORS_H

typedef struct {
  bool boot_okay;
  bool (*init_func)();
  const char *name;
} sensor_t;

#define NUM_SENSORS 7

bool sensors_boot_okay();
const char* sensors_failed_sensor();
void sensors_init();

#endif // ROBOCUP_SENSORS_H
