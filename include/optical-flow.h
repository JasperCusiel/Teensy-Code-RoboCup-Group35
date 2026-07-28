//
// Created by Jasper Cusiel on 21/07/2026.
//

#ifndef ROBOCUP_OPTICAL_FLOW_H
#define ROBOCUP_OPTICAL_FLOW_H

#include <stdbool.h>
bool optical_flow_init();
void flow_get_velocity(float *vx, float *vy);
void compensate_flow(float *vx, float *vy, float omega);

#endif // ROBOCUP_OPTICAL_FLOW_H
