//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIMIT_SWITCH_H
#define ROBOCUP_LIMIT_SWITCH_H

#include <stdint.h>

// Initialize limit switches on SX1509 IO expander
bool limit_switches_init();

// Read limit switch state (with debouncing)
bool read_limit_switch(uint8_t pin);

#endif // ROBOCUP_LIMIT_SWITCH_H
