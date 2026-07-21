//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_LIFT_MOTOR_H
#define ROBOCUP_LIFT_MOTOR_H

bool lifter_motor_init();

void pwm_skip_tuning();
void serialPrintPWMSkipResults();


void serialPrintAccuracyResults();
void accuracy_estimate_tuning();

void pid_tuning();

#endif // ROBOCUP_LIFT_MOTOR_H
