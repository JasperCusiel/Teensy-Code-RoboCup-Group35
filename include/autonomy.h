//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_AUTONOMY_H
#define ROBOCUP_AUTONOMY_H

// Starts autonomy task and sets safe output (stopped robot) until first planning cycle is complete.
void autonomy_init();

// Mission,planning,path-following and VFH checking task
void autonomy_task();

// Fast motion control update loop.
void autonomy_motion_task();

#endif // ROBOCUP_AUTONOMY_H
