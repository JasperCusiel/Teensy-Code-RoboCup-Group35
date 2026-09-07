//
// Created by Jasper Cusiel on 21/07/2026.
//
#include <inductive-sensor.h>
#include <core_pins.h>

#define SENSE_PIN A8

bool inductive_sensor_init()
{
    // Set pin mode and check output is sensible for no weight in front of sensor.
    pinMode(SENSE_PIN, INPUT);
    if (digitalRead(SENSE_PIN))
    {
        return true;
    }
    return false;
}
