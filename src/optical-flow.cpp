//
// Created by Jasper Cusiel on 21/07/2026.
//

#include <optical-flow.h>
#include <Bitcraze_PMW3901.h>

#define OPTICAL_FLOW_CS 10

Bitcraze_PMW3901 flow(OPTICAL_FLOW_CS);

bool optical_flow_init() {
  for (int i = 0; i < 20; i++) {  // 2 seconds max
    if (flow.begin() == true) {
      return true;
    }

    delay(100);
  }
  return false;
}