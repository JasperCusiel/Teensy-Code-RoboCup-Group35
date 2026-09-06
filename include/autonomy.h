//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_AUTONOMY_H
#define ROBOCUP_AUTONOMY_H

void autonomy_init();
// Mission/planning/path-following/VFH task. Run at approximately 20 Hz.
void autonomy_task();
// Fast motor-control task. Run near the odometry rate (approximately 95 Hz).
void autonomy_motion_task();

#endif // ROBOCUP_AUTONOMY_H
