//
// Created by Jasper Cusiel on 21/07/2026.
//

#include "optical-flow.h"
#include <Bitcraze_PMW3901.h>

#define OPTICAL_FLOW_CS 10
#define HEIGHT 0.09f // Mounting height off floor [m]

// Physical sensor offset from robot center.
#define FLOW_OFFSET_X 0.0f    // [m]
#define FLOW_OFFSET_Y (-0.150f)    // [m]

Bitcraze_PMW3901 flow(OPTICAL_FLOW_CS);

uint32_t last_time = 0; // For calculating dt

bool optical_flow_init()
{
    // Wait for sensor to show on I2C bus.
    for (int i = 0; i < 20; i++)
    {
        // 2 seconds max
        if (flow.begin() == true)
        {
            return true;
        }

        delay(100);
    }
    // Call to clear motion count and set last call time.
    flow.readMotionCount(nullptr, nullptr);
    last_time = millis();

    return false;
}

void flow_get_velocity(float* vx, float* vy, float dt)
{
    int16_t dx, dy;
    flow.readMotionCount(&dx, &dy);
    // From: https://github.com/PX4/PX4-Autopilot/blob/fe80e7aa468a50bec6b035d0e8e4e37e516c84ff/src/drivers/optical_flow/pmw3901/PMW3901.cpp
    float flow_rate_x = (float)dx / 385.0f; // proportional factor + convert from pixels to radians
    float flow_rate_y = (float)dy / 385.0f; // proportional factor + convert from pixels to radians

    // Robot frame:
    // +X = right
    // +Y = forward
    *vx = -flow_rate_x / dt * HEIGHT;
    *vy = flow_rate_y / dt * HEIGHT;
}

// Compensate for sensor offset using gyro
void compensate_flow(float* vx, float* vy, float omega)
{
    *vx += omega * FLOW_OFFSET_Y;
    *vy -= omega * FLOW_OFFSET_X;
}
