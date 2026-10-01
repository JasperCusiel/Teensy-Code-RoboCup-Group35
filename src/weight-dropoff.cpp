#include "weight-dropoff.h"
#include "mission.h"
#include "lift-motor.h"
#include "smart-servo.h"
#include "motion-controller.h"
#include "drivetrain.h"
#include "odometry.h"
#include "math_utils.h"
#include <Arduino.h>
#include <math.h>

namespace
{
    constexpr float kDockHeadingRad = 0.0f; // Forward +Y, reverse toward bottom wall.
    constexpr float kDockHeadingToleranceRad = 0.0873f; // Five degrees.
    constexpr float kDockHeadingKp = 2.5f;
    constexpr float kDockMinimumTurnRate = 0.5f;
    constexpr float kDockMaximumTurnRate = 0.9f;
    constexpr uint32_t kAlignTimeoutMs = 15000;
    constexpr uint32_t kAlignStableMs = 300;
    constexpr float kDockReverseSpeedMps = -0.1f;
    constexpr uint32_t kDockReverseDurationMs = 3000; // How long to reverse against wall for
    constexpr uint32_t kDockSettleMs = 300;
    constexpr uint32_t kLowerTimeoutMs = 15000;
    constexpr float kUnloadSpeedMps = 0.07f;
    constexpr uint32_t kUnloadDurationMs = 5000; // Preserves existing forward unloading.
    constexpr int kDockMaximumPwm = 255;

    weight_dropoff_state_t state = DROPOFF_STATUS_IDLE;
    uint32_t entered_ms = 0, aligned_since_ms = 0;
    bool aligned = false;

    void enter(weight_dropoff_state_t next)
    {
        state = next;
        entered_ms = millis();
    }

    void release_drive()
    {
        motion_controller_release_override(MotionOverrideOwner::DROPOFF);
        drivetrain_set_output_limit(255);
    }

    void abort_dropoff(const char* reason)
    {
        motion_controller_stop();
        lifter_stop();
        if (state == DROPOFF_STATUS_UNLOAD) set_back_servo_down();
        release_drive();
        state = DROPOFF_STATUS_IDLE;
        mission_stop(); // Do not repeatedly reattempt a failed docking sequence.
        Serial.print("DOCK STOP: ");
        Serial.println(reason);
    }
}

void weight_dropoff_state_update()
{
    // Dropoff sequence FSM
    if (mission_get_state() != MISSION_COMPLETE)
    {
        if (state != DROPOFF_STATUS_IDLE)
        {
            motion_controller_stop();
            lifter_stop();
            if (state == DROPOFF_STATUS_UNLOAD) set_back_servo_down();
            release_drive();
            state = DROPOFF_STATUS_IDLE;
        }
        return;
    }

    if (state == DROPOFF_STATUS_DONE) return; // Wait for mission to consume completion.

    if (drivetrain_get_fault() != DrivetrainFault::NONE)
    {
        abort_dropoff("drivetrain fault");
        return;
    }

    if (!motion_controller_acquire_override(MotionOverrideOwner::DROPOFF))
    {
        abort_dropoff("drive unavailable");
        return;
    }

    const uint32_t now = millis();
    const uint32_t elapsed = now - entered_ms;

    switch (state)
    {
    case DROPOFF_STATUS_IDLE:
        motion_controller_stop();
        aligned = false;
        enter(DROPOFF_STATUS_LOWER);
        Serial.println("DOCK: align to zero heading");
        break;

    // case DROPOFF_STATUS_ALIGN:
    //     {
    //         float x, y, heading;
    //         get_ekf_pose(&x, &y, &heading);
    //         if (!isfinite(heading))
    //         {
    //             abort_dropoff("invalid heading");
    //             return;
    //         }
    //         if (elapsed >= kAlignTimeoutMs)
    //         {
    //             abort_dropoff("alignment timeout");
    //             return;
    //         }
    //         const float error = wrap_angle_rad(kDockHeadingRad - heading);
    //         if (fabsf(error) <= kDockHeadingToleranceRad)
    //         {
    //             motion_controller_stop();
    //             if (!aligned)
    //             {
    //                 aligned = true;
    //                 aligned_since_ms = now;
    //             }
    //             if (now - aligned_since_ms >= kAlignStableMs)
    //             {
    //                 drivetrain_set_output_limit(kDockMaximumPwm);
    //                 motion_controller_set_override_heading(kDockHeadingRad);
    //                 enter(DROPOFF_STATUS_REVERSE);
    //                 motion_controller_override_drive(kDockReverseSpeedMps, 0.0f);
    //                 Serial.println("DOCK: timed reverse");
    //             }
    //         }
    //         else
    //         {
    //             aligned = false;
    //             float rate = fminf(kDockMaximumTurnRate,
    //                                fmaxf(kDockMinimumTurnRate, kDockHeadingKp * fabsf(error)));
    //             if (error < 0) rate = -rate;
    //             motion_controller_override_drive(0.0f, rate);
    //         }
    //         break;
    //     }

    // case DROPOFF_STATUS_REVERSE:
    //     if (elapsed >= kDockReverseDurationMs)
    //     {
    //         motion_controller_stop(); // Zero wheel targets reset both wheel PI integrators.
    //         drivetrain_set_output_limit(255);
    //         enter(DROPOFF_STATUS_SETTLE);
    //         Serial.println("DOCK: reverse ended; settle");
    //     }
    //     else
    //     {
    //         motion_controller_override_drive(kDockReverseSpeedMps, 0.0f);
    //     }
    //     break;

    // case DROPOFF_STATUS_SETTLE:
    //     motion_controller_stop();
    //     if (elapsed >= kDockSettleMs)
    //     {
    //         lifter_lower();
    //         enter(DROPOFF_STATUS_LOWER);
    //         Serial.println("DOCK: lower lifter");
    //     }
    //     break;

    case DROPOFF_STATUS_LOWER:
        motion_controller_stop();
        if (elapsed >= kLowerTimeoutMs)
        {
            abort_dropoff("lifter timeout");
            return;
        }
        if (is_lifter_reached_target())
        {
            set_back_servo_up();
            motion_controller_set_override_heading(kDockHeadingRad);
            enter(DROPOFF_STATUS_UNLOAD);
            motion_controller_override_drive(kUnloadSpeedMps, 0.0f);
            Serial.println("DOCK: unload moving forward");
        }
        break;

    case DROPOFF_STATUS_UNLOAD:
        if (elapsed >= kUnloadDurationMs)
        {
            motion_controller_stop();
            set_back_servo_down();
            release_drive();
            enter(DROPOFF_STATUS_DONE);
            mission_report_dropoff_complete(true); // Sequence completed; not contact/weight verification.
            Serial.println("DOCK: sequence complete");
        }
        else
        {
            motion_controller_override_drive(kUnloadSpeedMps, 0.0f);
        }
        break;

    case DROPOFF_STATUS_DONE:
        break;
    }
}
