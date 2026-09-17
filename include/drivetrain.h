//
// Created by Jasper Cusiel on 06/09/2026.
//

#ifndef ROBOCUP_DRIVETRAIN_H
#define ROBOCUP_DRIVETRAIN_H

void drivetrain_init();

// Run the drivetrain speed controllers. Call frequently from the scheduler.
void drivetrain_update();

// Set wheel speed targets in metres per second.
void set_wheel_speed_targets(float left_mps, float right_mps);

// Backwards-compatible wrapper. These values are wheel speed targets in m/s,
// not normalized motor effort.
void set_motor_speeds(float left_mps, float right_mps);
void PID_tune();

#endif // ROBOCUP_DRIVETRAIN_H
