//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIMIT_SWITCH_H
#define ROBOCUP_LIMIT_SWITCH_H

#include <stdint.h>
#include <wiring.h>

// SX1509 Pins:
const byte SX1509_AIO0 = 0;
const byte SX1509_AIO1 = 1;
const byte SX1509_AIO2 = 2;
const byte SX1509_AIO3 = 3;
const byte SX1509_AIO4 = 4;
const byte SX1509_AIO5 = 5;
const byte SX1509_AIO6 = 6;
const byte SX1509_AIO7 = 7;
const byte SX1509_AIO8 = 8;
const byte SX1509_AIO9 = 9;
const byte SX1509_AIO10 = 10;
const byte SX1509_AIO11 = 11;
const byte SX1509_AIO12 = 12;
const byte SX1509_AIO13 = 13;
const byte SX1509_AIO14 = 14;
const byte SX1509_AIO15 = 15;

// Initialize limit switches on SX1509 IO expander
bool limit_switches_init();

// Read limit switch state (with debouncing)
bool read_limit_switch(uint8_t pin);

bool read_io_pin(uint8_t pin);

#endif // ROBOCUP_LIMIT_SWITCH_H
