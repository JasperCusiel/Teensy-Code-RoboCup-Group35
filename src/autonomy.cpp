//
// Created by Jasper Cusiel on 03/09/2026.
//

#include "autonomy.h"
#include "mission.h"
#include "motion-controller.h"
#include "navigation.h"
#include "pure-pursuit.h"
#include "vfh.h"
#include "weight-pickup.h"
#include "odometry.h"
#include <math.h>
#include "math_utils.h"

namespace // Keep variables and helper functions private to this file.
{
    constexpr float kPi = 3.14159265f;
    constexpr float kRecoveryHeadingOffsetRad = 1.0f;
    constexpr float kObstacleTurnRateRadPerSec = 2.5f;
    constexpr float kRecoveryRejectAngleRad = 1.5f * kPi;
    constexpr float kRecoveryExitSpeedMps = 0.10f;
    constexpr float kTurnInPlaceSpeedScale = 0.50f;
    constexpr float kSlowFrontClearanceM = 0.35f;
    constexpr float kHardFrontClearanceM = 0.08f;
    constexpr float kExplorationScanTurnRateRadPerSec = 1.0f;
    constexpr float kWeightApproachSpeedMps = 0.08f;
    constexpr float kSteeringDirectionDeadbandRad = 0.05f;
    constexpr float kTurnInPlaceLinearDeadbandMps = 0.01f;
    constexpr float kTurnInPlaceTurnRateDeadbandRadPerSec = 0.01f;
    constexpr float kAvoidanceReplanDeflectionRad = 0.55f;
    constexpr uint8_t kAvoidanceReplanCycles = 8;
    constexpr uint8_t kAvoidanceReplanCooldownCycles = 20;
    constexpr bool kHeadingHoldTestMode = false;
    constexpr float kHeadingHoldTestSpeedMps = 0.1f;
    pose_t current_pose = {}; // Stores the latest EKF pose estimate
    velocity_command_t safe_command = {0.0f, 0.0f, 0.0f, true};
    // Most recent command passed through the obstical avoidance.
    float last_steering_direction = 1.0f;
    // Records if the last VFH turn direction was left (+1) or right (-1), used to choose escape direction to spin when VFH can't find a free direction in the FOV.
    float recovery_turn_angle_rad = 0.0f;
    float recovery_last_heading_rad = 0.0f;
    bool recovery_turn_started = false;
    uint8_t avoidance_replan_cycles = 0;
    uint8_t avoidance_replan_cooldown_cycles = 0;
    bool heading_hold_test_started = false;
    float heading_hold_test_heading = 0.0f;

    void reset_recovery_turn_tracking()
    {
        recovery_turn_angle_rad = 0.0f;
        recovery_last_heading_rad = 0.0f;
        recovery_turn_started = false;
    }

    void record_recovery_turn(float heading)
    {
        if (!recovery_turn_started)
        {
            recovery_last_heading_rad = heading;
            recovery_turn_started = true;
            return;
        }

        recovery_turn_angle_rad += fabsf(wrap_angle_rad(heading - recovery_last_heading_rad));
        recovery_last_heading_rad = heading;

        if (recovery_turn_angle_rad >= kRecoveryRejectAngleRad)
        {
            navigation_reject_current_goal();
            reset_recovery_turn_tracking();
        }
    }

    void reset_avoidance_replan_tracking()
    {
        avoidance_replan_cycles = 0;
        avoidance_replan_cooldown_cycles = 0;
    }

    void update_avoidance_replan_tracking(bool avoidance_active)
    {
        if (avoidance_replan_cooldown_cycles > 0)
        {
            --avoidance_replan_cooldown_cycles;
        }

        if (!navigation_has_path())
        {
            avoidance_replan_cycles = 0;
            return;
        }

        if (avoidance_active)
        {
            if (avoidance_replan_cycles < kAvoidanceReplanCycles)
            {
                ++avoidance_replan_cycles;
            }
        }
        else if (avoidance_replan_cycles > 0)
        {
            --avoidance_replan_cycles;
        }

        if (avoidance_replan_cycles >= kAvoidanceReplanCycles &&
            avoidance_replan_cooldown_cycles == 0)
        {
            navigation_request_replan();
            avoidance_replan_cycles = 0;
            avoidance_replan_cooldown_cycles = kAvoidanceReplanCooldownCycles;
        }
    }

    bool is_turn_in_place_command(const velocity_command_t& command)
    {
        return fabsf(command.linear_speed) <= kTurnInPlaceLinearDeadbandMps &&
            fabsf(command.turn_rate) > kTurnInPlaceTurnRateDeadbandRadPerSec;
    }

    velocity_command_t avoid_obstacles(const velocity_command_t& target,
                                       const pose_t& pose,
                                       bool following_path)
    {
        // Bypass VFH if stop is commanded (prevents VFH trying to command movement)
        if (target.stop)
        {
            reset_recovery_turn_tracking();
            reset_avoidance_replan_tracking();
            return target;
        }

        // Convert world frame target heading (from pure pursuit) into VFH robot frame.
        const float target_relative = wrap_angle_rad(target.heading - pose.theta);
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
            update_avoidance_replan_tracking(true);
            record_recovery_turn(pose.theta);
            return {
                wrap_angle_rad(pose.theta + recovery_direction * kRecoveryHeadingOffsetRad),
                0.0f,
                (recovery_direction * kObstacleTurnRateRadPerSec),
                false
            };
        }

        if (is_turn_in_place_command(target))
        {
            const bool exit_recovery =
                following_path &&
                recovery_turn_started &&
                forward_clearance > kHardFrontClearanceM;
            reset_recovery_turn_tracking();
            reset_avoidance_replan_tracking();
            last_steering_direction = target.turn_rate > 0.0f ? 1.0f : -1.0f;
            if (exit_recovery)
            {
                float speed = kRecoveryExitSpeedMps;
                if (forward_clearance < kSlowFrontClearanceM)
                {
                    const float clearance_scale =
                        (forward_clearance - kHardFrontClearanceM) /
                        (kSlowFrontClearanceM - kHardFrontClearanceM);
                    speed *= fminf(1.0f, fmaxf(0.0f, clearance_scale));
                }

                return {
                    wrap_angle_rad(pose.theta + steering_relative),
                    speed,
                    0.0f,
                    false
                };
            }
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
        safe.heading = wrap_angle_rad(pose.theta + steering_relative);
        const float deflection = fabsf(wrap_angle_rad(steering_relative - target_relative));

        // Slow down based on obstical avoidance deflection amount, larger VFH avoidance -> slower speed.
        // At 90 degree or more, turn in place instead of arc.
        const float speed_scale = fmaxf(0.0f, cosf(deflection));
        const bool avoidance_active =
            deflection >= kAvoidanceReplanDeflectionRad ||
            forward_clearance < kSlowFrontClearanceM ||
            speed_scale <= kTurnInPlaceSpeedScale;
        update_avoidance_replan_tracking(avoidance_active);
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
            record_recovery_turn(pose.theta);
        }
        else
        {
            safe.turn_rate = 0.0f;
            reset_recovery_turn_tracking();
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

    velocity_command_t heading_hold_test_command(const pose_t& pose)
    {
        if (!heading_hold_test_started)
        {
            heading_hold_test_heading = pose.theta;
            heading_hold_test_started = true;
        }

        return {
            heading_hold_test_heading,
            kHeadingHoldTestSpeedMps,
            0.0f,
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
    reset_recovery_turn_tracking();
    reset_avoidance_replan_tracking();
    heading_hold_test_started = false;
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

    if (kHeadingHoldTestMode)
    {
        safe_command = mission_should_stop()
                           ? velocity_command_t{current_pose.theta, 0.0f, 0.0f, true}
                           : heading_hold_test_command(current_pose);
        return;
    }

    // Select or update navigation goal and path
    navigation_task();

    if (mission_get_state() == MISSION_WEIGHT_DETECTED)
    {
        //safe_command = should_drive_towards_weight()
        //? weight_approach_command(current_pose)
        //: velocity_command_t{current_pose.theta, 0.0f, 0.0f, true};
        return;
    }

    // Get heading and forward speed from pure pursuit
    const path_t* path = navigation_get_path();
    const velocity_command_t target =
        mission_should_explore() && path == nullptr
            ? exploration_scan_command(current_pose)
            : pure_pursuit_update(path, &current_pose);
    // Pass through VFH obstacle avoidance
    safe_command = avoid_obstacles(target, current_pose, path != nullptr);
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
    if (weight_pickup_get_state() == PICKUP_STATUS_IDLE)
    {
        motion_controller_update(&current_pose, &safe_command);
    }
}
