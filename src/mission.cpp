//
// Created by Jasper Cusiel on 03/09/2026.
//
#include "mission.h"

#include "button.h"
#include "navigation.h"
#include "colour-sensor.h"

// Mission module handles the mission logic via FSM to
// determine the robots behavior -> IDLE, EXPLORE, WEIGHT_DETECTED, RETURN_HOME, etc.

namespace
{
    // Functions and variables private to module.
    constexpr uint8_t kTargetWeightCount = 3; // Number of weights to collect before returning home
    constexpr uint8_t kRecoveryFailuresBeforeReturnHome = 3;

    mission_state_t state = MISSION_IDLE; //MISSION IDLE
    uint8_t collected_weight_count = 0;
    uint8_t recovery_failure_count = 0;
    bool weight_detected_event = false;
    bool pickup_complete_event = false;
    bool pickup_succeeded = false;
    bool dropoff_succeeded = false;
    bool dropoff_complete_event = false;

    void enter_state(mission_state_t new_state)
    {
        if (state == new_state)
        {
            return;
        }
        state = new_state;

        // Clear or request replan if we are in any of the four states below.
        // Keeps navigation synchronized with mission state
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

    void reject_or_clear_navigation_goal()
    {
        if (navigation_has_goal())
        {
            navigation_reject_current_goal();
        }
        else
        {
            navigation_clear_goal();
        }
    }

    void handle_exploration_path_failure()
    {
        reject_or_clear_navigation_goal();
        if (recovery_failure_count < kRecoveryFailuresBeforeReturnHome)
        {
            ++recovery_failure_count;
        }

        enter_state(recovery_failure_count >= kRecoveryFailuresBeforeReturnHome
                        ? MISSION_RETURN_HOME
                        : MISSION_RECOVERING);
    }
} // namespace

void mission_init()
{
    state = MISSION_IDLE;
    collected_weight_count = 0;
    recovery_failure_count = 0;
    weight_detected_event = false;
    pickup_complete_event = false;
    pickup_succeeded = false;
    dropoff_complete_event = false;
    dropoff_succeeded = false;
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
        else if (navigation_path_failed())
        {
            handle_exploration_path_failure();
        }
        else if (navigation_exploration_complete())
        {
            // Coverage is complete, return to the saved base cell.
            recovery_failure_count = 0;
            enter_state(MISSION_RETURN_HOME);
        }
        else if (navigation_has_path())
        {
            recovery_failure_count = 0;
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
            recovery_failure_count = 0;
            enter_state(collected_weight_count >= kTargetWeightCount ? MISSION_RETURN_HOME : MISSION_EXPLORE);
        }
        break;

    case MISSION_RECOVERING:
        // Keep exploration alive after a failed target: scan, pick another goal,
        // and only give up on exploring after repeated recovery failures.
        if (weight_detected_event)
        {
            weight_detected_event = false;
            recovery_failure_count = 0;
            enter_state(MISSION_WEIGHT_DETECTED);
        }
        else if (navigation_path_failed())
        {
            handle_exploration_path_failure();
        }
        else if (navigation_exploration_complete())
        {
            recovery_failure_count = 0;
            enter_state(MISSION_RETURN_HOME);
        }
        else if (navigation_has_path())
        {
            recovery_failure_count = 0;
            enter_state(MISSION_EXPLORE);
        }
        break;

    case MISSION_RETURN_HOME:
        // If home is temporarily unreachable, keep clearing/replanning. Autonomy
        // will spin-scan while there is no valid path.
        if (navigation_path_failed())
        {
            navigation_clear_goal();
        }
        // Enter idle state once mission is complete (at home)
        else if (navigation_goal_reached() && (get_current_color() == get_base_color()))
        {
            recovery_failure_count = 0;
            enter_state(MISSION_COMPLETE);
        }
        break;

    case MISSION_COMPLETE:
        if (dropoff_complete_event)
        {
            // Clear flag
            dropoff_complete_event = false;
            if (dropoff_succeeded)
            {
                collected_weight_count = 0;
                enter_state(MISSION_EXPLORE);
            }
            else
            {
                enter_state(MISSION_RETURN_HOME);
            }
        }
        break;

    case MISSION_STOPPED:

        break;
    }
}

mission_state_t mission_get_state() { return state; }


// Helper functions provide intent signals to naviagtion and autonomy without exposing transition logic.
bool mission_should_explore()
{
    return state == MISSION_EXPLORE || state == MISSION_RECOVERING;
}

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
    if (mission_should_explore())
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

void mission_report_dropoff_complete(bool success)
{
    if (state == MISSION_COMPLETE)
    {
        dropoff_succeeded = success;
        dropoff_complete_event = true;
    }
}

uint8_t mission_get_weight_count() { return collected_weight_count; }

void mission_return_home()
{
    enter_state(MISSION_RETURN_HOME);
}
