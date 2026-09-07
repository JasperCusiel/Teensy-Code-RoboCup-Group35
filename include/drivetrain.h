//
// Created by Jasper Cusiel on 06/09/2026.
//

#ifndef ROBOCUP_DRIVETRAIN_H
#define ROBOCUP_DRIVETRAIN_H

void drivetrain_init();

// Set normalized motor speeds, values outside [-1,1] are clamped. -1 = full reverse, 1=full foward.
void set_motor_speeds(float left, float right);

#endif // ROBOCUP_DRIVETRAIN_H
