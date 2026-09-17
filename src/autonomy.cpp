//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "autonomy.h"
#include "mission.h"
#include "motion-controller.h"
#include "navigation.h"
#include "pure-pursuit.h"
#include "vfh.h"
#include "odometry.h"
#include <math.h>

namespace // Keep variables and helper functions private to this file.
{
    constexpr float kRecoveryHeadingOffsetRad = 1.0f;
    constexpr float kObstacleTurnRateRadPerSec = 2.0f;
    constexpr float kTurnInPlaceSpeedScale = 0.50f;
    constexpr float kSlowFrontClearanceM = 0.35f;
    constexpr float kHardFrontClearanceM = 0.08f;
    constexpr float kExplorationScanTurnRateRadPerSec = 0.5f;
    constexpr uint8_t kRecoveryCyclesBeforeFrontierReject = 40;
    constexpr float kSteeringDirectionDeadbandRad = 0.05f;
    constexpr float kTurnInPlaceLinearDeadbandMps = 0.01f;
    constexpr float kTurnInPlaceTurnRateDeadbandRadPerSec = 0.01f;
    pose_t current_pose = {}; // Stores the latest EKF pose estimate
    velocity_command_t safe_command = {0.0f, 0.0f, 0.0f, true};
    // Most recent command passed through the obstical avoidance.
    float last_steering_direction = 1.0f;
    // Records if the last VFH turn direction was left (+1) or right (-1), used to choose escape direction to spin when VFH can't find a free direction in the FOV.
    uint8_t recovery_turn_cycles = 0;

    // Normalize heading angles so difference is always smallest rotation.
    float wrap_angle(float angle)
    {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    void record_recovery_turn()
    {
        if (recovery_turn_cycles < kRecoveryCyclesBeforeFrontierReject)
        {
            ++recovery_turn_cycles;
        }

        if (recovery_turn_cycles >= kRecoveryCyclesBeforeFrontierReject)
        {
            navigation_reject_current_goal();
            recovery_turn_cycles = 0;
        }
    }

    bool is_turn_in_place_command(const velocity_command_t& command)
    {
        return fabsf(command.linear_speed) <= kTurnInPlaceLinearDeadbandMps &&
            fabsf(command.turn_rate) > kTurnInPlaceTurnRateDeadbandRadPerSec;
    }

    velocity_command_t avoid_obstacles(const velocity_command_t& target, const pose_t& pose)
    {
        // Bypass VFH if stop is commanded (prevents VFH trying to command movement)
        if (target.stop)
        {
            recovery_turn_cycles = 0;
            return target;
        }

        // Convert world frame target heading (from pure pursuit) into VFH robot frame.
        const float target_relative = wrap_angle(target.heading - pose.theta);
        vfh_set_target_angle(target_relative);

        // Run obstacle avoidance every cycle so it can't be bypassed by pure pursuit path follower
        compute_vfh();
        // Select collision free steering direction
        const float steering_relative = vfh_get_steering_angle();
        const float forward_clearance = vfh_get_forward_clearance();

        // No free direction if steering vfh direction is NAN
        if (!isfinite(steering_relative))
        {
            // Keep rotating the same way so recovery does not oscillate left/right.
            const float recovery_direction = last_steering_direction;
            record_recovery_turn();
            return {
                wrap_angle(pose.theta + recovery_direction * kRecoveryHeadingOffsetRad),
                0.0f,
                recovery_direction * kObstacleTurnRateRadPerSec,
                false
            };
        }

        if (is_turn_in_place_command(target))
        {
            recovery_turn_cycles = 0;
            last_steering_direction = target.turn_rate > 0.0f ? 1.0f : -1.0f;
            return target;
        }

        // Only consider meaningful steering values as turn commands.
        if (fabsf(steering_relative) > kSteeringDirectionDeadbandRad)
        {
            last_steering_direction = steering_relative > 0.0f ? 1.0f : -1.0f;
        }

        // Target command has been verified to be "safe"
        velocity_command_t safe = target;
        // Convert robot frame back to world frame
        safe.heading = wrap_angle(pose.theta + steering_relative);
        const float deflection = fabsf(wrap_angle(steering_relative - target_relative));

        // Slow down based on obstical avoidance deflection amount, larger VFH avoidance -> slower speed.
        // At 90 degree or more, turn in place instead of arc.
        const float speed_scale = fmaxf(0.0f, cosf(deflection));
        safe.linear_speed *= speed_scale;
        if (forward_clearance < kSlowFrontClearanceM)
        {
            const float clearance_scale =
                (forward_clearance - kHardFrontClearanceM) /
                (kSlowFrontClearanceM - kHardFrontClearanceM);
            safe.linear_speed *= fminf(1.0f, fmaxf(0.0f, clearance_scale));
        }

        if (speed_scale <= kTurnInPlaceSpeedScale ||
            forward_clearance <= kHardFrontClearanceM)
        {
            const float turn_direction =
                fabsf(steering_relative) > kSteeringDirectionDeadbandRad
                    ? (steering_relative > 0.0f ? 1.0f : -1.0f)
                    : last_steering_direction;
            safe.linear_speed = 0.0f;
            safe.turn_rate = turn_direction * kObstacleTurnRateRadPerSec;
            record_recovery_turn();
        }
        else
        {
            safe.turn_rate = 0.0f;
            recovery_turn_cycles = 0;
        }
        // A zero forward speed is still a valid rotate-in-place command.
        safe.stop = false;

        return safe;
    }

    velocity_command_t exploration_scan_command(const pose_t& pose)
    {
        return {
            pose.theta,
            0.0f,
            last_steering_direction * kExplorationScanTurnRateRadPerSec,
            false
        };
    }
} // namespace

void autonomy_init()
{
    // Initialize the current pose (from the EKF), mission and nav tasks.
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
    mission_init();
    navigation_init();
    motion_controller_init();
    last_steering_direction = 1.0f;
    recovery_turn_cycles = 0;
    // Publish stop command until first planning cycle produces a safe motion command.
    safe_command = {current_pose.theta, 0.0f, 0.0f, true};
}

void autonomy_task()
{
    // Slow autonomy loop -> advances mission state, updates path, follows it and filters output through local obstical avoidance.


    // Get lastest robot pose
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);

    // Advance the mission state
    mission_task();

    // Select or update navigation goal and path
    navigation_task();

    // Get heading and forward speed from pure pursuit
    const path_t* path = navigation_get_path();
    const velocity_command_t target =
        mission_should_explore() && path == nullptr
            ? exploration_scan_command(current_pose)
            : pure_pursuit_update(path, &current_pose);
    // Pass through VFH obstacle avoidance
    safe_command = avoid_obstacles(target, current_pose);
    // Override if mission state commands stop.
    if (mission_should_stop())
    {
        safe_command = {current_pose.theta, 0.0f, 0.0f, true};
    }
}

void autonomy_motion_task()
{
    // Fast autonomy loop -> get latest pose and update motion controller with latest command.
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
    motion_controller_update(&current_pose, &safe_command);
}
