//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIFT_MOTOR_H
#define ROBOCUP_LIFT_MOTOR_H

#include <DCMotorServo.h>
#include <Servo.h>
#include <Encoder.h>

bool lifter_motor_init();
bool home_servo(DCMotorServo *servo);

void pwm_skip_tuning(Servo *motor, uint8_t pwm_pin, Encoder *encoder, void (*motorBreak)(), void (*motorWrite)(int16_t s));
void serialPrintPWMSkipResults();


void serialPrintAccuracyResults();
void accuracy_estimate_tuning();

void pid_tuning();

#endif // ROBOCUP_LIFT_MOTOR_H
