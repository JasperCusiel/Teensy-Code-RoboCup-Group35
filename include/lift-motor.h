//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIFT_MOTOR_H
#define ROBOCUP_LIFT_MOTOR_H
#include <DCMotorServo.h>

// Initializes motor drive, encoders, and homes both lifter motors sequentially.
bool lifter_motor_init();
void lifter_motor_update();
void lifter_raise();
void lifter_lower();
void lifter_move_middle();
void lifter_stop();
bool is_lifter_reached_target();
long lifter_get_position1();
long lifter_get_position2();

#endif // ROBOCUP_LIFT_MOTOR_H
