//
// Created by Jasper Cusiel on 06/09/2026.
//

#ifndef ROBOCUP_DRIVETRAIN_H
#define ROBOCUP_DRIVETRAIN_H

constexpr float kDrivetrainMaxWheelSpeedMps = 0.30f;

void drivetrain_init();

// Set wheel speed targets in metres per second.
void set_open_loop_wheel_speed_targets(float left_mps, float right_mps);


#endif // ROBOCUP_DRIVETRAIN_H
