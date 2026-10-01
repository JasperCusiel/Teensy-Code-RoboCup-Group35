//
// Created by Joe Elder on 02/09/2026
//

#ifndef ROBOCUP_IR_REFLECTIVE_H
#define ROBOCUP_IR_REFLECTIVE_H

#include <stdint.h>

// Initalize sensor
bool ir_reflective_sensor_init();

// Sample sensor with moving average, 
void ir_reflective_update();

// return true if average value is greater than threshold (weight present) or false if not.
bool is_weight_detected_ir_reflective();

// Get raw IR reading value
int16_t get_ir_reading();

#endif // ROBOCUP_IR_REFLECTIVE_H
