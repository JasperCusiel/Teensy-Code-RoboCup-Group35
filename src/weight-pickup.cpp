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
#include <math.h>

#define WEIGHT_TYPE_CHECK_CYCLES 10
#define ALIGNING_TIMEOUT_MS 10000
#define FAKE_WEIGHT_CLEAR_DELAY_MS 500
#define LOADING_WEIGHT_DELAY_MS 500
#define END_STOP_A_PIN 0
#define END_STOP_B_PIN 1
#define WEIGHT_UNDETECTED_TIMEOUT_MS 5000
#define SEARCHING_TIMEOUT_MS 5000

static weight_pickup_state_t current_state = PICKUP_STATUS_IDLE;
static short weight_type_check_cycle = 0;
static unsigned long aligning_start_time = 0;
static int weight_pos = 0;
static unsigned long fake_weight_clear_start = 0;
static unsigned long loading_confirm_start = 0;
static unsigned long weight_undetected_time = 0;
static unsigned long searching_start_time = 0;



void weight_pickup_state_update()
{
    switch (current_state) {
    case PICKUP_STATUS_IDLE:
      if (mission_get_state() == MISSION_WEIGHT_DETECTED) {
        aligning_start_time = millis();
        current_state = PICKUP_STATUS_ALIGNING;
        }
        break;

    case PICKUP_STATUS_ALIGNING:
        if (is_weight_detected_ir_reflective()) {
            //slow down drive straight
            current_state = PICKUP_STATUS_CHECKING_WEIGHT_TYPE;
            break;
        }
        if (millis() - aligning_start_time > ALIGNING_TIMEOUT_MS) { //timeout
            // stop 
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
            break;
        }

        int weight_pos = get_weight_position();       //to do --------------------------------------------------------------------

        if (weight_pos == LEFT) {
            //turn left
            weight_undetected_time = 0;
        } else if (weight_pos == RIGHT) {
            //turn right
            weight_undetected_time = 0;
        } else if (weight_pos == CENTRE){         // if in middle or no longer in FOV
            //move forward
            weight_undetected_time = 0;
        } else if (weight_pos == LOST) {
            //move forward slowly
            if (weight_undetected_time == 0) {
                weight_undetected_time = millis();
            } else if (millis() - weight_undetected_time > WEIGHT_UNDETECTED_TIMEOUT_MS) {
                // been stuck in "undetected" for too long, give up on aligning
                weight_undetected_time = 0;
                searching_start_time = millis();
                current_state = PICKUP_STATUS_WEIGHT_LOST;
                break;
            }
        }
        break;

    case PICKUP_STATUS_WEIGHT_LOST:
        int weight_pos = get_weight_position(); 
        if  (millis() - searching_start_time > SEARCHING_TIMEOUT_MS) {
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
            break;
        }
        if (weight_pos != LOST) {
            //stop turning
            searching_start_time = 0;
            current_state = PICKUP_STATUS_ALIGNING;
            break;
        } else {
            //turn on spot
        }
        break;




    case PICKUP_STATUS_CHECKING_WEIGHT_TYPE:
        if (is_real_weight_inductive_sensor()) {
            weight_type_check_cycle = 0;
            //stop
            current_state = PICKUP_STATUS_LOWERING_RAILS;
            break;
        }
        weight_type_check_cycle++;
        if (weight_type_check_cycle > WEIGHT_TYPE_CHECK_CYCLES) {
            weight_type_check_cycle = 0;
            current_state = PICKUP_STATUS_FAKE_WEIGHT_DETECTED;
        }
        break;


    case PICKUP_STATUS_FAKE_WEIGHT_DETECTED:
        set_front_servo_up();
        current_state = PICKUP_STATUS_FAKE_WEIGHT_LIFTING;
        break;

    case PICKUP_STATUS_FAKE_WEIGHT_LIFTING:
        if (is_front_servo_in_position()) {
       // drive forward to clear weight
            current_state = PICKUP_STATUS_FAKE_WEIGHT_MOVING;
        }
        break;

    case PICKUP_STATUS_FAKE_WEIGHT_MOVING:
        if (!is_weight_detected_ir_reflective()) {
            fake_weight_clear_start = millis();
            current_state = PICKUP_STATUS_FAKE_WEIGHT_CLEAR_DELAY;
        }
        break;

    case PICKUP_STATUS_FAKE_WEIGHT_CLEAR_DELAY:
        if (millis() - fake_weight_clear_start > FAKE_WEIGHT_CLEAR_DELAY_MS) {
            set_front_servo_down();
            mission_report_pickup_complete(false);
            current_state = PICKUP_STATUS_IDLE;
        }
        break;

    case PICKUP_STATUS_LOWERING_RAILS:
        if (read_limit_switch(END_STOP_A_PIN) && read_limit_switch(END_STOP_B_PIN)) {
            lifter_stop();
            current_state = PICKUP_STATUS_LOADING_WEIGHT;
            break;
        }
        lifter_lower();
        set_front_servo_up();
        break;


    case PICKUP_STATUS_LOADING_WEIGHT:
        if (!is_weight_detected_ir_reflective()) {
            loading_confirm_start = millis();
            current_state = PICKUP_STATUS_LOADING_WEIGHT_CONFIRM_DELAY;
            break;
        }
        //drive forwards            
        break;

    case PICKUP_STATUS_LOADING_WEIGHT_CONFIRM_DELAY:
        if (millis() - loading_confirm_start > LOADING_WEIGHT_DELAY_MS ) {
            set_front_servo_down();
            current_state = PICKUP_STATUS_LIFTING_WEIGHT;
        }
        break;



    case PICKUP_STATUS_LIFTING_WEIGHT:
        if (is_lifter_reached_top()) {
            mission_report_pickup_complete(true);
            current_state = PICKUP_STATUS_IDLE;
        }
        lifter_raise();
        break;
    }
}

