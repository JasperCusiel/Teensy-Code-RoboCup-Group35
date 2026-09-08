//
// Created by Jasper Cusiel on 03/09/2026.
//
#include "mission.h"
#include "navigation.h"

// Mission module handles the mission logic via FSM to
// determine the robots behavior -> IDLE, EXPLORE, WEIGHT_DETECTED, RETURN_HOME, etc.

namespace
{
    // Functions and variables private to module.
    constexpr uint8_t kTargetWeightCount = 3; // Number of weights to collect before returning home

    mission_state_t state = MISSION_IDLE;
    uint8_t collected_weight_count = 0;
    bool weight_detected_event = false;
    bool pickup_complete_event = false;
    bool pickup_succeeded = false;

    void enter_state(mission_state_t new_state)
    {
        if (state == new_state)
        {
            return;
        }
        state = new_state;

        // Clear or request replan if we are in any of the four states below.
        // Keeps navigation synchornised with mission state
        if (state == MISSION_IDLE || state == MISSION_WEIGHT_DETECTED ||
            state == MISSION_COMPLETE || state == MISSION_STOPPED)
        {
            // Resets all mission progress and pending event flags set back to startup state.
            navigation_clear_goal();
        }
        else
        {
            navigation_request_replan();
        }
    }
} // namespace

void mission_init()
{
    state = MISSION_IDLE;
    collected_weight_count = 0;
    weight_detected_event = false;
    pickup_complete_event = false;
    pickup_succeeded = false;
}

void mission_task()
{
    // Advances mission state machine.
    switch (state)
    {
    case MISSION_IDLE:
        // Nothing happens, wait until mission start called (transition to MISSION_EXPLORE)
        break;

    case MISSION_EXPLORE:
        // React to highest priority event (weight detection).
        // Weight detected, enter weight detection state, clear event.
        if (weight_detected_event)
        {
            weight_detected_event = false;
            enter_state(MISSION_WEIGHT_DETECTED);
        }
        else if (navigation_path_failed() || navigation_exploration_complete())
        {
            // Stop if nav failure or no where left to explore.
            enter_state(MISSION_STOPPED);
        }

        break;

    case MISSION_WEIGHT_DETECTED:
        // Pickup runs outside mission module, once it reports completion, count successful pickup and
        // decide to continue exploring or return home.
        if (pickup_complete_event)
        {
            // Clear flag
            pickup_complete_event = false;
            if (pickup_succeeded && collected_weight_count < kTargetWeightCount)
            {
                ++collected_weight_count;
            }
            // Return home only entered after collecting target number of weights.
            enter_state(collected_weight_count >= kTargetWeightCount ? MISSION_RETURN_HOME : MISSION_EXPLORE);
        }
        break;

    case MISSION_RETURN_HOME:
        // Stop if we cant navigate home.
        if (navigation_path_failed())
        {
            enter_state(MISSION_STOPPED);
        }
        // Enter idle state once mission is complete (at home)
        else if (navigation_goal_reached())
        {
            enter_state(MISSION_COMPLETE);
        }
        break;

    case MISSION_COMPLETE:
    case MISSION_STOPPED:
        break;
    }
}

mission_state_t mission_get_state() { return state; }


// Helper functions provide intent signals to naviagtion and autonomy without exposing transition logic.
bool mission_should_explore() { return state == MISSION_EXPLORE; }

bool mission_should_return_home() { return state == MISSION_RETURN_HOME; }

bool mission_should_stop()
{
    return state == MISSION_IDLE || state == MISSION_WEIGHT_DETECTED ||
        state == MISSION_COMPLETE || state == MISSION_STOPPED;
}

void mission_start()
{
    if (state == MISSION_IDLE)
    {
        enter_state(MISSION_EXPLORE);
    }
}

void mission_stop() { enter_state(MISSION_STOPPED); }

void mission_reset()
{
    navigation_clear_goal();
    mission_init();
}

// Latch external events only when they are valid for the current mission state.
// mission_task() consumes flags to keep transitions deterministic.
void mission_report_weight_detected()
{
    if (state == MISSION_EXPLORE)
    {
        weight_detected_event = true;
    }
}

void mission_report_pickup_complete(bool success)
{
    if (state == MISSION_WEIGHT_DETECTED)
    {
        pickup_succeeded = success;
        pickup_complete_event = true;
    }
}

uint8_t mission_get_weight_count() { return collected_weight_count; }
