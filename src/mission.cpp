//
// Created by Jasper Cusiel on 03/09/2026.
//
#include "mission.h"

#include "button.h"
#include "navigation.h"
#include "colour-sensor.h"
#include "odometry.h"
#include <Arduino.h>

// Mission module handles the mission logic via FSM to
// determine the robots behavior -> IDLE, EXPLORE, WEIGHT_DETECTED, RETURN_HOME, etc.
//
// Rule: the robot only leaves exploration for RETURN_HOME once it is carrying
// kTargetWeightCount weights. Path failures and "coverage complete" never end
// exploration on their own; they just trigger recovery / another search pass.

namespace
{
    // Functions and variables private to module.
    constexpr uint8_t kTargetWeightCount = 3; // Weights onboard before returning home
    constexpr uint32_t kFrontierExplorationDurationMs = 20000;
    constexpr uint32_t kExplorationRetryIntervalMs = 1000; // Min gap between "keep searching" replans
    constexpr float kBaseXGreen = 0.4f;
    constexpr float kBaseXOther = 2.6f;
    constexpr float kBaseY = 0.4f;
    constexpr float kReturnNudgeStepM = 0.10f;
    constexpr float kReturnNudgeMinDistM = 0.05f;

    mission_state_t state = MISSION_IDLE;
    uint8_t collected_weight_count = 0;
    uint8_t recovery_failure_count = 0; // Diagnostic / escalation counter, saturates at 255
    bool weight_detected_event = false;
    bool pickup_complete_event = false;
    bool pickup_succeeded = false;
    bool dropoff_complete_event = false;
    bool dropoff_succeeded = false;
    uint32_t mission_started_ms = 0;
    uint32_t last_search_replan_ms = 0;
    bool mission_started = false;

    bool have_full_load()
    {
        return collected_weight_count >= kTargetWeightCount;
    }

    void enter_state(mission_state_t new_state)
    {
        if (state == new_state)
        {
            return;
        }
        state = new_state;

        // Clear the goal in states where the robot should be stationary,
        // otherwise request a replan so navigation follows the new state.
        if (state == MISSION_IDLE || state == MISSION_WEIGHT_DETECTED ||
            state == MISSION_COMPLETE || state == MISSION_STOPPED)
        {
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

    // Keep searching for weights without leaving exploration.
    // Rate limited so a "complete" coverage map doesn't replan every tick.
    void keep_searching()
    {
        const uint32_t now = millis();
        if (static_cast<uint32_t>(now - last_search_replan_ms) < kExplorationRetryIntervalMs)
        {
            return;
        }
        last_search_replan_ms = now;

        // TODO: if navigation keeps reporting "complete" with a full coverage map,
        // reset visited cells here so the arena is swept again.
        navigation_request_replan();
    }

    // Shared by EXPLORE and RECOVERING when navigation reports coverage complete.
    void handle_exploration_complete()
    {
        recovery_failure_count = 0;
        if (have_full_load())
        {
            enter_state(MISSION_RETURN_HOME);
        }
        else
        {
            keep_searching();
        }
    }

    // Path failed while exploring: never give up on exploring, just recover.
    void handle_exploration_path_failure()
    {
        reject_or_clear_navigation_goal();
        if (recovery_failure_count < UINT8_MAX)
        {
            ++recovery_failure_count;
        }

        if (state == MISSION_RECOVERING)
        {
            // enter_state() is a no-op when already RECOVERING, so replan explicitly.
            navigation_request_replan();
        }
        else
        {
            enter_state(MISSION_RECOVERING);
        }
    }

    void keep_returning_to_base()
    {
        pose_t pose = {};
        get_ekf_pose(&pose.x, &pose.y, &pose.theta);

        const float base_x = (get_base_color() == COLOR_GREEN) ? kBaseXGreen : kBaseXOther;
        const float dx = base_x - pose.x;
        const float dy = kBaseY - pose.y;
        const float length = hypotf(dx, dy);

        if (length < kReturnNudgeMinDistM)
        {
            return;
        }

        const float target_x = pose.x + (dx / length) * kReturnNudgeStepM;
        const float target_y = pose.y + (dy / length) * kReturnNudgeStepM;

        navigation_set_base(target_x, target_y);
        navigation_request_replan();
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
    mission_started_ms = 0;
    last_search_replan_ms = 0;
    mission_started = false;
}

void mission_task()
{
    // Advances mission state machine.
    switch (state)
    {
    case MISSION_IDLE:
        // Wait until mission_start() (transition to MISSION_EXPLORE).
        break;

    case MISSION_EXPLORE:
        // Highest priority: weight detection.
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
            handle_exploration_complete();
        }
        else if (navigation_has_path())
        {
            recovery_failure_count = 0;
        }
        break;

    case MISSION_WEIGHT_DETECTED:
        // Pickup runs outside the mission module; once it reports completion,
        // count the weight and either keep exploring or head home.
        if (pickup_complete_event)
        {
            pickup_complete_event = false;
            if (pickup_succeeded && collected_weight_count < kTargetWeightCount)
            {
                ++collected_weight_count;
            }
            recovery_failure_count = 0;
            enter_state(have_full_load() ? MISSION_RETURN_HOME : MISSION_EXPLORE);
        }
        break;

    case MISSION_RECOVERING:
        // Keep exploration alive after a failed target: pick another goal and
        // go back to EXPLORE as soon as a path exists.
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
            handle_exploration_complete();
        }
        else if (navigation_has_path())
        {
            recovery_failure_count = 0;
            enter_state(MISSION_EXPLORE);
        }
        break;

    case MISSION_RETURN_HOME:
        // If nav thinks we've reached the home target but we can't see the base
        // colour yet, keep nudging toward the home corner.
        if (navigation_goal_reached())
        {
            if (get_current_color() == get_base_color())
            {
                recovery_failure_count = 0;
                enter_state(MISSION_COMPLETE);
            }
            else
            {
                keep_returning_to_base();
            }
            break;
        }

        if (navigation_path_failed())
        {
            navigation_clear_goal();
        }
        break;

    case MISSION_COMPLETE:
        if (dropoff_complete_event)
        {
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

// Helper functions provide intent signals to navigation and autonomy without
// exposing transition logic.
bool mission_should_explore()
{
    return state == MISSION_EXPLORE || state == MISSION_RECOVERING;
}

bool mission_should_use_coverage()
{
    return mission_started &&
           static_cast<uint32_t>(millis() - mission_started_ms) >=
               kFrontierExplorationDurationMs;
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
        mission_started_ms = millis();
        mission_started = true;
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
// mission_task() consumes the flags to keep transitions deterministic.
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

// Only honoured with a full load, so external callers can't bypass the rule.
void mission_return_home()
{
    if (have_full_load())
    {
        enter_state(MISSION_RETURN_HOME);
    }
}
