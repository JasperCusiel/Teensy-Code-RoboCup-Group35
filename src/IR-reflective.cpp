//
// Created by Joe Elder on 02/09/2026.
//

#include <IR-reflective.h>
#include <core_pins.h>

#define SENSE_PIN A13
#define SENSE_THRESHOLD 30
#define SAMPLE_COUNT 3

static int16_t buffer[3] = {0, 0, 0};
static uint8_t buf_index = 0;
static int16_t average = 0;


bool ir_reflective_sensor_init()
{
    // Start sensor
    pinMode(SENSE_PIN, INPUT);
    return true;
}

void ir_reflective_update()
{
    // Moving average three readings
    buffer[buf_index] = analogRead(SENSE_PIN);
    buf_index = (buf_index + 1) % 3;
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) sum += buffer[i];
    average = sum / SAMPLE_COUNT;
}


bool is_weight_detected_ir_reflective()
{
    // Average readings and check if over threshold to determine if weight is present.
    return (average > SENSE_THRESHOLD);
}
