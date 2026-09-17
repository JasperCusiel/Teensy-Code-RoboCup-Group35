//
// Created by Jasper Cusiel on 17/09/2026.
//

#include "heading-encoder.h"
#include "limit-switch.h"
#include <Arduino.h>

// Define input pins for the 8421 encoder bits.
static const int pinBit1 = SX1509_AIO13; // 2^0 (1)
static const int pinBit2 = SX1509_AIO12; // 2^1 (2)
static const int pinBit4 = SX1509_AIO14; // 2^2 (4)
static const int pinBit8 = SX1509_AIO15; // 2^3 (8)


float heading_encoder_get_value()
{
    const int bit1 = read_io_pin(pinBit1);
    const int bit2 = read_io_pin(pinBit2);
    const int bit4 = read_io_pin(pinBit4);
    const int bit8 = read_io_pin(pinBit8);

    const int encoder_value = (bit8 << 3) | (bit4 << 2) | (bit2 << 1) | bit1;

    // Calculate angle in radians (2 * PI / 16 steps = PI / 8)
    return encoder_value * (PI / 8.0f);
}
