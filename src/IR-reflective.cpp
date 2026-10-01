//
// Created by Joe Elder on 02/09/2026.
//

#include <IR-reflective.h>
#include <core_pins.h>
#include <Arduino.h>

#define SENSE_PIN A13
#define SENSE_THRESHOLD 10
#define SAMPLE_COUNT 3

static int16_t buffer[3] = {0, 0, 0};
static uint8_t buf_index = 0;
static int16_t average = 0;

// Raw and filtered IR readings for right/left sensors
static int16_t right_ir_reading_raw = 0;
static int16_t left_ir_reading_raw = 0;
static float right_ir_filtered = 0.0f;
static float left_ir_filtered = 0.0f;

// EMA smoothing factor (0..1). Larger => faster response, smaller => smoother.
static const float kIRAlpha = 0.35f;


bool ir_reflective_sensor_init()
{
    // Start sensor
    pinMode(SENSE_PIN, INPUT);
    pinMode(A0, INPUT);
    pinMode(A10, INPUT);
    return true;
}

void ir_reflective_update()
{
    // Moving average three readings
    buffer[buf_index] = analogRead(SENSE_PIN);
    buf_index = (buf_index + 1) % SAMPLE_COUNT;
    
    int32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) sum += buffer[i];
    average = sum / SAMPLE_COUNT;
    // Read raw sensor values
    int16_t raw_right = analogRead(A0);
    int16_t raw_left = analogRead(A10);

    right_ir_reading_raw = raw_right;
    left_ir_reading_raw = raw_left;

    // Update exponential moving average filters
    if (right_ir_filtered == 0.0f) right_ir_filtered = raw_right; // init on first sample
    else right_ir_filtered = (1.0f - kIRAlpha) * right_ir_filtered + kIRAlpha * raw_right;

    if (left_ir_filtered == 0.0f) left_ir_filtered = raw_left; // init on first sample
    else left_ir_filtered = (1.0f - kIRAlpha) * left_ir_filtered + kIRAlpha * raw_left;
}


bool is_weight_detected_ir_reflective()
{
    // Average readings and check if over threshold to determine if weight is present.
    return (average > SENSE_THRESHOLD);
}

int16_t get_right_ir_reading()
{
    return static_cast<int16_t>(right_ir_filtered + 0.5f);
}

int16_t get_left_ir_reading()
{
    return static_cast<int16_t>(left_ir_filtered + 0.5f);
}