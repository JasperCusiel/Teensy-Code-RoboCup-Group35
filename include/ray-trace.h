//
// Created by Jasper Cusiel on 05/08/2026.
//

#ifndef ROBOCUP_RAY_TRACE_H
#define ROBOCUP_RAY_TRACE_H

typedef void (*ray_callback_t)(int x, int y);

void ray_cast(int x0, int y0, int x1, int y1, ray_callback_t fill_callback, ray_callback_t end_point_callback);

#endif // ROBOCUP_RAY_TRACE_H
