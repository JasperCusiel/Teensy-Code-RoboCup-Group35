//
// Created by Jasper Cusiel on 21/07/2026.
//

#include <optical-flow.h>
#include <Bitcraze_PMW3901.h>

#define OPTICAL_FLOW_CS 10
#define FLOW_SCALE 0.0209 // [rad/count] -> scale = FOV/resolution = 42 deg / 35 pix
#define HEIGHT 0.08f // mounting height off floor [m]

// Physical sensor offset from robot center.
#define FLOW_OFFSET_X 0.05f    // [m]
#define FLOW_OFFSET_Y 0.0f    // [m]

Bitcraze_PMW3901 flow(OPTICAL_FLOW_CS);

uint32_t last_time = 0; // For calculating dt

bool optical_flow_init() {
  for (int i = 0; i < 20; i++) {  // 2 seconds max
    if (flow.begin() == true) {
      return true;
    }

    delay(100);
  }
  // call to clear motion count and set last call time.
  flow.readMotionCount(nullptr, nullptr);
  last_time = millis();

  return false;
}

void flow_get_velocity(float *vx, float *vy) {
  int16_t dx, dy;
  flow.readMotionCount(&dx, &dy);

  uint32_t current_time = millis();
  uint32_t dt = millis() - last_time;

  last_time = current_time;

  *vx = dx * HEIGHT * FLOW_SCALE / dt;
  *vy = dy * HEIGHT * FLOW_SCALE / dt;
}

// Compensate for sensor offset
void compensate_flow(float *vx, float *vy, float omega)
{
  *vx += omega * FLOW_OFFSET_Y;
  *vy -= omega * FLOW_OFFSET_X;
}
