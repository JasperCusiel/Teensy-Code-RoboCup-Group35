//
// Created by Joe Elder on 02/09/2026.
//

#include <IR-reflective.h>
#include <core_pins.h>

#define SENSE_PIN A13
#define SENSE_THRESHOLD 30

static int16_t buffer[3] = {0, 0, 0};
static uint8_t buf_index = 0;
static int16_t average = 0;


bool ir_reflective_sensor_init()
{
    // Start sensor
    pinMode(SENSE_PIN, INPUT);
    return true;
}

static void update_average()
{
    // Moving average three readings
    buffer[buf_index] = analogRead(SENSE_PIN);
    average = (buffer[0] + buffer[1] + buffer[2]) / 3;
    buf_index = (buf_index + 1) % 3;
}


bool ir_reflective_weight_detected()
{
    // Average readings and check if over threshold to determine if weight is present.
    update_average();
    return (average > SENSE_THRESHOLD);
}
