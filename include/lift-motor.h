//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIFT_MOTOR_H
#define ROBOCUP_LIFT_MOTOR_H
#include <DCMotorServo.h>

// Initializes motor drive, encoders, and homes both lifter motors sequentially.
bool lifter_motor_init();

// Homes servo by backing off, performing a fast then slow homing sequence.
bool home_servo(DCMotorServo* servo);

#endif // ROBOCUP_LIFT_MOTOR_H
