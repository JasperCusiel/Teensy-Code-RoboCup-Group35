//
// Created by Jasper Cusiel on 03/09/2026.
//

#ifndef ROBOCUP_MISSION_H
#define ROBOCUP_MISSION_H

#include <stdint.h>

// FSM states
typedef enum
{
    MISSION_IDLE,
    MISSION_EXPLORE,
    MISSION_WEIGHT_DETECTED,
    MISSION_RETURN_HOME,
    MISSION_COMPLETE,
    MISSION_STOPPED
} mission_state_t;

// Sets initial mission to IDLE
void mission_init();

// Call periodically to update mission FSM
void mission_task();

// Returns mission FSM state
mission_state_t mission_get_state();

// Returns true if in EXPLORE state
bool mission_should_explore();

// Returns true if in RETURN_HOME state.
bool mission_should_return_home();

// Returns true if in IDLE, WEIGHT_DETECTED, COMPLETE, or STOPPED state.
bool mission_should_stop();

// Enters EXPLORE state
void mission_start();

// Enters STOPPED state
void mission_stop();

// Clears nav goal and resets mission state to boot state.
void mission_reset();

// Latches weight detected event for FSM to handle.
void mission_report_weight_detected();

// Transitions out of WEIGHT_DETECTED state.
void mission_report_pickup_complete(bool success);

// Return number of successfully collected weights.
uint8_t mission_get_weight_count();

#endif // ROBOCUP_MISSION_H
