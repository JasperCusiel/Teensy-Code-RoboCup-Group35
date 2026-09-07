//
// Created by Joe Elder on 02/09/2026
//

#ifndef ROBOCUP_IR_REFLECTIVE_H
#define ROBOCUP_IR_REFLECTIVE_H

// Initalize sensor
bool ir_reflective_sensor_init();

// Sample sensor with moving average, return true if average value is greater than
// threshold (weight present) or false if not.
bool ir_reflective_weight_detected();

#endif // ROBOCUP_IR_REFLECTIVE_H
