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
#include "drivetrain.h"
#include <math.h>
#include "math_utils.h"
#include <Arduino.h>

namespace // Keep variables and helper functions private to this file.
{
    constexpr float kPi = 3.14159265f;
    constexpr float kRecoveryHeadingOffsetRad = 1.5f;
    constexpr float kObstacleTurnRateRadPerSec = 2.5f;
    constexpr float kRecoveryRejectAngleRad = 1.5f * kPi;
    constexpr float kRecoveryExitSpeedMps = 0.10f;
    constexpr float kTurnInPlaceSpeedScale = 0.750f;
    constexpr float kSlowFrontClearanceM = 0.35f;
    constexpr float kHardFrontClearanceM = 0.08f;
    constexpr float kExplorationScanTurnRateRadPerSec = 0.5f;
    constexpr float kWeightApproachSpeedMps = 0.08f;
    constexpr float kSteeringDirectionDeadbandRad = 0.05f;
    constexpr float kTurnInPlaceLinearDeadbandMps = 0.01f;
    constexpr float kTurnInPlaceTurnRateDeadbandRadPerSec = 0.01f;
    constexpr float kAvoidanceReplanDeflectionRad = 0.55f;
    constexpr uint8_t kAvoidanceReplanCycles = 8;
    constexpr uint8_t kAvoidanceReplanCooldownCycles = 20;
    constexpr bool kHeadingHoldTestMode = false;
    constexpr float kHeadingHoldTestSpeedMps = 0.1f;
    float scan_start_x = 0.0f;
    float scan_start_y = 0.0f;

    constexpr float kScanResetDistanceM = 0.10f;

    // Virtual target distance used when following a VFH-selected bearing.
    // Start with the same distance as the pure-pursuit lookahead.
    constexpr float kAvoidanceLookaheadM = 0.50f;

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

    constexpr float kScanTurnRateRadPerSec = 0.50f;
    constexpr uint32_t kScanTimeoutMs = 30000;
    constexpr float kScanReverseSpeedMps = -0.15f;
    constexpr float kReverseMotionThresholdMps = 0.01f;
    constexpr uint8_t kReverseMotionConfirmations = 2;
    constexpr float kReverseRecoveryDistanceM = 0.30f;
    constexpr uint32_t kReverseRetryPauseMs = 500;
    uint32_t reverse_retry_started_ms = 0;
    uint32_t last_motion_debug_ms = 0;

    bool scan_active = false;
    uint32_t scan_started_ms = 0;
    float scan_direction = 1.0f;
    bool reverse_recovery_active = false;
    bool reverse_retry_pending = false;
    bool reverse_motion_confirmed = false;
    uint8_t reverse_motion_confirmations = 0;

    void reset_scan()
    {
        scan_active = false;
        reverse_recovery_active = false;
        reverse_retry_pending = false;
        reverse_motion_confirmed = false;
        reverse_motion_confirmations = 0;
    }

    void begin_reverse_recovery(const pose_t& pose)
    {
        scan_active = true;
        scan_start_x = pose.x;
        scan_start_y = pose.y;
        reverse_recovery_active = true;
        reverse_retry_pending = false;
        reverse_motion_confirmed = false;
        reverse_motion_confirmations = 0;
    }

    bool reverse_motion_detected()
    {
        DrivetrainTelemetry telemetry = {};
        drivetrain_get_telemetry(&telemetry);

        return telemetry.fault == DrivetrainFault::NONE &&
            telemetry.left_measured_mps <= -kReverseMotionThresholdMps &&
            telemetry.right_measured_mps <= -kReverseMotionThresholdMps;
    }

    velocity_command_t bounded_scan_command(const pose_t& pose)
    {
        if (!scan_active)
        {
            scan_active = true;
            scan_started_ms = millis();

            scan_start_x = pose.x;
            scan_start_y = pose.y;

            scan_direction = last_steering_direction >= 0.0f
                                 ? 1.0f
                                 : -1.0f;
        }

        if (reverse_recovery_active)
        {
            if (!reverse_motion_confirmed && reverse_motion_detected())
            {
                if (reverse_motion_confirmations < kReverseMotionConfirmations)
                {
                    ++reverse_motion_confirmations;
                }
            }
            else if (!reverse_motion_confirmed)
            {
                reverse_motion_confirmations = 0;
            }

            if (!reverse_motion_confirmed &&
                reverse_motion_confirmations >= kReverseMotionConfirmations)
            {
                reverse_motion_confirmed = true;
                Serial.println(
                    "RECOVERY: reverse motion detected; continuing 0.30 m");
            }

            const float reverse_dx = pose.x - scan_start_x;
            const float reverse_dy = pose.y - scan_start_y;
            if (reverse_motion_confirmed &&
                reverse_dx * reverse_dx + reverse_dy * reverse_dy >=
                kReverseRecoveryDistanceM * kReverseRecoveryDistanceM)
            {
                reset_scan();
                navigation_request_replan();
                Serial.println("RECOVERY: reverse distance reached; replanning");
                return {pose.theta, 0.0f, 0.0f, true};
            }

            return {pose.theta, kScanReverseSpeedMps, 0.0f, false};
        }

        if (static_cast<uint32_t>(millis() - scan_started_ms) >=
            kScanTimeoutMs)
        {
            begin_reverse_recovery(pose);

            Serial.println("RECOVERY: scan timed out; reversing");

            return {pose.theta, kScanReverseSpeedMps, 0.0f, false};
        }

        return {
            pose.theta,
            0.0f,
            scan_direction * kScanTurnRateRadPerSec,
            false
        };
    }

    void reset_recovery_turn_tracking()
    {
        recovery_turn_angle_rad = 0.0f;
        recovery_last_heading_rad = 0.0f;
        recovery_turn_started = false;
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
            update_avoidance_replan_tracking(true);
            return bounded_scan_command(pose);
        }

        if (is_turn_in_place_command(target))
        {
            reset_avoidance_replan_tracking();

            last_steering_direction =
                target.turn_rate > 0.0f ? 1.0f : -1.0f;

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
            return bounded_scan_command(pose);
        }
        else
        {
            // If VFH substantially changed the target bearing, generate
            // curvature toward that new bearing.
            if (deflection > kSteeringDirectionDeadbandRad)
            {
                const float curvature =
                    2.0f * sinf(steering_relative) /
                    kAvoidanceLookaheadM;

                safe.turn_rate =
                    safe.linear_speed * curvature;
            }
            else
            {
                // VFH accepted approximately the original direction.
                // Keep pure pursuit's curvature when reducing speed.
                const float speed_scale =
                    fabsf(target.linear_speed) > 0.001f
                        ? safe.linear_speed / target.linear_speed
                        : 0.0f;

                safe.turn_rate =
                    target.turn_rate * speed_scale;
            }

            reset_recovery_turn_tracking();
        }
        // A zero forward speed is still a valid rotate-in-place command.
        safe.stop = false;

        return safe;
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
    reset_scan();
}

void autonomy_task()
{
    // Slow autonomy loop -> advances mission state, updates path, follows it and filters output through local obstical avoidance.


    // Get lastest robot pose
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);

    // Advance the mission state
    mission_task();

    const DrivetrainFault drivetrain_fault = drivetrain_get_fault();
    const bool recoverable_stall =
        drivetrain_fault == DrivetrainFault::LEFT_STALLED ||
        drivetrain_fault == DrivetrainFault::RIGHT_STALLED;
    const bool mission_allows_recovery =
        mission_should_explore() || mission_should_return_home();

    // A latched stall zeros and rejects every wheel command. Clear it while
    // stopped, then immediately replace the failed command with reverse.
    if (recoverable_stall && mission_allows_recovery &&
        !reverse_recovery_active)
    {
        if (reverse_retry_pending &&
            static_cast<uint32_t>(millis() - reverse_retry_started_ms) <
            kReverseRetryPauseMs)
        {
            safe_command = {current_pose.theta, 0.0f, 0.0f, true};
            return;
        }

        const bool retrying = reverse_retry_pending;
        motion_controller_stop();
        drivetrain_clear_fault();
        begin_reverse_recovery(current_pose);
        safe_command = {
            current_pose.theta,
            kScanReverseSpeedMps,
            0.0f,
            false
        };
        Serial.println(retrying
                           ? "RECOVERY: retrying reverse after stall"
                           : "RECOVERY: drivetrain stall cleared; reversing");
        return;
    }

    // If reverse itself stalls, pause at neutral before clearing the stall and
    // trying again. Competition operation must not require physical access.
    if (reverse_recovery_active &&
        drivetrain_fault != DrivetrainFault::NONE)
    {
        reverse_recovery_active = false;
        safe_command = {current_pose.theta, 0.0f, 0.0f, true};
        if (recoverable_stall && mission_allows_recovery)
        {
            reverse_retry_pending = true;
            reverse_retry_started_ms = millis();
            Serial.println("RECOVERY: reverse stalled; pausing before retry");
        }
        else
        {
            reverse_retry_pending = false;
            Serial.println("RECOVERY: reverse hit a non-recoverable drive fault");
        }
        return;
    }

    if (drivetrain_fault != DrivetrainFault::NONE)
    {
        safe_command = {current_pose.theta, 0.0f, 0.0f, true};
        return;
    }

    // if (!check_navigation_progress(current_pose))
    // {
    //     safe_command = {
    //         current_pose.theta,
    //         0.0f,
    //         0.0f,
    //         true
    //     };
    //
    //     return;
    // }

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
        safe_command = {
            current_pose.theta,
            0.0f,
            0.0f,
            true
        };
        reset_scan();
        return;
    }

    // Get heading and forward speed from pure pursuit
    const path_t* path = navigation_get_path();

    velocity_command_t target = {
        current_pose.theta, 0.0f, 0.0f, true
    };

    if (mission_should_stop())
    {
        reset_scan();
        safe_command = target;
    }
    else if (reverse_recovery_active)
    {
        // Finish the reverse even if navigation now has a path.
        target = bounded_scan_command(current_pose);
        safe_command = target;
    }
    else if ((mission_should_explore() || mission_should_return_home()) &&
        path == nullptr)
    {
        // Run recovery only once this cycle.
        target = bounded_scan_command(current_pose);
        safe_command = target;
    }
    else
    {
        target = pure_pursuit_update(path, &current_pose);
        safe_command = avoid_obstacles(
            target, current_pose, path != nullptr);
    }

    if (static_cast<uint32_t>(
        millis() - last_motion_debug_ms) >= 500)
    {
        last_motion_debug_ms = millis();

        Serial.printf(
            "CMD PP: stop=%u v=%.3f w=%.3f | "
            "SAFE: stop=%u v=%.3f w=%.3f\n",
            target.stop ? 1u : 0u,
            target.linear_speed,
            target.turn_rate,
            safe_command.stop ? 1u : 0u,
            safe_command.linear_speed,
            safe_command.turn_rate);
        Serial.printf(
            "MISSION state=%u | NAV state=%u path=%u "
            "scan=%u reverse=%u steer=%.1f clearance=%.2f\n",
            static_cast<unsigned>(mission_get_state()),
            static_cast<unsigned>(navigation_get_status()),
            navigation_has_path() ? 1u : 0u,
            scan_active ? 1u : 0u,
            reverse_recovery_active ? 1u : 0u,
            vfh_get_steering_angle() * 180.0f / PI,
            vfh_get_forward_clearance());
    }
    // Override if mission state commands stop.
    if (mission_should_stop())
    {
        safe_command = {current_pose.theta, 0.0f, 0.0f, true};
    }

    // Re-arm only after measured translation away from
    // the position where this recovery attempt started.
    if (scan_active &&
        !safe_command.stop &&
        safe_command.linear_speed > 0.02f)
    {
        const float dx = current_pose.x - scan_start_x;
        const float dy = current_pose.y - scan_start_y;

        if (dx * dx + dy * dy >=
            kScanResetDistanceM * kScanResetDistanceM)
        {
            reset_scan();
        }
    }
}

void autonomy_motion_task()
{
    if (mission_get_state() == MISSION_STOPPED ||
        mission_get_state() == MISSION_IDLE)
    {
        motion_controller_stop();
        return;
    }
    // Fast autonomy loop -> get latest pose and update motion controller with latest command.
    get_ekf_pose(&current_pose.x, &current_pose.y, &current_pose.theta);
    if (weight_pickup_get_state() == PICKUP_STATUS_IDLE)
    {
        motion_controller_update(&current_pose, &safe_command);
    }
}
