//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_MISSION_H
#define ROBOCUP_MISSION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MISSION_IDLE,
  MISSION_EXPLORE,
  MISSION_WEIGHT_DETECTED,
  MISSION_RETURN_HOME,
  MISSION_COMPLETE,
  MISSION_STOPPED
} mission_state_t;

void mission_init();
void mission_task();

mission_state_t mission_get_state();
bool mission_should_explore();
bool mission_should_return_home();
bool mission_should_stop();

void mission_start();
void mission_stop();
void mission_reset();

void mission_report_weight_detected();
void mission_report_pickup_complete(bool success);
uint8_t mission_get_weight_count();

#endif // ROBOCUP_MISSION_H
