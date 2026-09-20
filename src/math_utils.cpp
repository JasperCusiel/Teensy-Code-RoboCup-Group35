//
// Created by Jasper Cusiel on 18/09/2026.
//
#include "math_utils.h"
#include <wiring.h>

// Normalize heading angles so difference is always smallest rotation.
float wrap_angle_rad(float angle_rad)
{
    while (angle_rad > PI) angle_rad -= 2.0f * PI;
    while (angle_rad < -PI) angle_rad += 2.0f * PI;
    return angle_rad;
}
