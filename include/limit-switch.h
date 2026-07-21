//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIMIT_SWITCH_H
#define ROBOCUP_LIMIT_SWITCH_H

#include <stdbool.h>
#include <stdint.h>

bool limit_switches_init();
bool readLimitSwitch(uint8_t pin);

#endif // ROBOCUP_LIMIT_SWITCH_H
