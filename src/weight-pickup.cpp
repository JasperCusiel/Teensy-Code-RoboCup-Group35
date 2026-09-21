//
// Created by Joe Elder on 02/09/2026.
//
#include "weight-pickup.h"
#include "ir-reflective.h"
#include "inductive-sensor.h"
#include "mission.h"
#include "drivetrain.h"
#include "core_pins.h"
#include "smart-servo.h"
#include "limit-switch.h"
#include "lift-motor.h"
#include "autonomy.h"
#include "navigation-types.h"
#include "motion-controller.h"
#include "odometry.h"
#include "weight-detection.h"
#include <math.h>

#define WEIGHT_TYPE_CHECK_CYCLES 10
#define ALIGNING_TIMEOUT_MS 10000
#define FAKE_WEIGHT_CLEAR_DELAY_MS 2000
#define LOADING_WEIGHT_DELAY_MS 2000
#define END_STOP_A_PIN 0
#define END_STOP_B_PIN 1
#define SEARCHING_TIMEOUT_MS 3000
#define LOST 0
#define LEFT 1
#define CENTRE 2
#define RIGHT 3
#define PICKUP_DRIVE_SPEED 0.03f

static weight_pickup_state_t current_state = PICKUP_STATUS_IDLE;
static short weight_type_check_cycle = 0;
static unsigned long aligning_start_time = 0;
static unsigned long fake_weight_clear_start = 0;
static unsigned long loading_confirm_start = 0;
static unsigned long last_weight_seen_time = 0;



void weight_pickup_state_update()
{
    switch (current_state)
    {
    case PICKUP_STATUS_IDLE:
        if (mission_get_state() == MISSION_WEIGHT_DETECTED)
        {
            aligning_start_time = millis();
            last_weight_seen_time = millis();
            current_state = PICKUP_STATUS_ALIGNING;
            break;
        }
        if (is_weight_detected_ir_reflective())
        {
            if (is_lifter_reached_target() && (mission_get_state() == MISSION_EXPLORE))
            {
                mission_report_weight_detected();
                motion_controller_override_drive(0.0f, 0.0f);
                current_state = PICKUP_STATUS_CHECKING_WEIGHT_TYPE;
            }
        }
        break;

    case PICKUP_STATUS_ALIGNING:
        if (is_weight_detected_ir_reflective())
        {
            motion_controller_override_drive(0.0f, 0.0f);
            current_state = PICKUP_STATUS_CHECKING_WEIGHT_TYPE;
            break;
        }
        if (millis() - aligning_start_time > ALIGNING_TIMEOUT_MS)
        {
            //timeout
            motion_controller_override_drive(0.0f, 0.0f);
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
            break;
        }
        if (detect_weight())
        {
            last_weight_seen_time = millis();
        }
        else if (millis() - last_weight_seen_time > SEARCHING_TIMEOUT_MS)
        {
            // haven't seen the weight for too long
            motion_controller_override_drive(0.0f, 0.0f);
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
            break;
        }
        motion_controller_override_drive(PICKUP_DRIVE_SPEED, 0.0f);
        break;

    case PICKUP_STATUS_CHECKING_WEIGHT_TYPE:
        motion_controller_override_drive(0.0f, 0.0f);
        if (is_real_weight_inductive_sensor())
        {
            weight_type_check_cycle = 0;
            motion_controller_override_drive(0.0f, 0.0f);
            current_state = PICKUP_STATUS_LOWERING_RAILS;
            break;
        }
        weight_type_check_cycle++;
        if (weight_type_check_cycle > WEIGHT_TYPE_CHECK_CYCLES)
        {
            weight_type_check_cycle = 0;
            current_state = PICKUP_STATUS_FAKE_WEIGHT_DETECTED;
        }
        break;


    case PICKUP_STATUS_FAKE_WEIGHT_DETECTED:
        set_front_servo_up();
        lifter_raise();
        current_state = PICKUP_STATUS_FAKE_WEIGHT_LIFTING;
        break;

    case PICKUP_STATUS_FAKE_WEIGHT_LIFTING:
        if (is_lifter_reached_target())
        {
            motion_controller_override_drive(PICKUP_DRIVE_SPEED, 0.0f);
            current_state = PICKUP_STATUS_FAKE_WEIGHT_MOVING;
        }
        break;


    case PICKUP_STATUS_FAKE_WEIGHT_MOVING:
        if (!is_weight_detected_ir_reflective())
        {
            fake_weight_clear_start = millis();
            current_state = PICKUP_STATUS_FAKE_WEIGHT_CLEAR_DELAY;
        }
        break;

    case PICKUP_STATUS_FAKE_WEIGHT_CLEAR_DELAY:
        if (millis() - fake_weight_clear_start > FAKE_WEIGHT_CLEAR_DELAY_MS)
        {
            set_front_servo_down();
            motion_controller_override_drive(0.0f, 0.0f);
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
        }
        break;

    case PICKUP_STATUS_LOWERING_RAILS:
        lifter_lower();
        motion_controller_override_drive(0.0f, 0.0f);
        if (is_lifter_reached_target())
        {
            set_front_servo_up();
    
            current_state = PICKUP_STATUS_LOADING_WEIGHT;
            break;
        }
        break;


    case PICKUP_STATUS_LOADING_WEIGHT:
        if (!is_weight_detected_ir_reflective())
        {
            loading_confirm_start = millis();
            current_state = PICKUP_STATUS_LOADING_WEIGHT_CONFIRM_DELAY;
            break;
        }
        motion_controller_override_drive(PICKUP_DRIVE_SPEED, 0.0f);
        break;

    case PICKUP_STATUS_LOADING_WEIGHT_CONFIRM_DELAY:
        if (millis() - loading_confirm_start > LOADING_WEIGHT_DELAY_MS)
        {
            set_front_servo_down();
            motion_controller_override_drive(0.0f, 0.0f);
            lifter_move_middle();
            current_state = PICKUP_STATUS_LIFTING_WEIGHT;
        }
        break;


    case PICKUP_STATUS_LIFTING_WEIGHT:
        if (is_lifter_reached_target())
        {
            mission_report_pickup_complete(true);
            current_state = PICKUP_STATUS_IDLE;
        }
        break;
    }
    motion_controller_set_override(current_state != PICKUP_STATUS_IDLE);
}


weight_pickup_state_t weight_pickup_get_state()
{
    return current_state;
}

