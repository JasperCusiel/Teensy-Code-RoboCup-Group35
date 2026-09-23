//
// Created by Joe Elder on 20/09/2026.
//

#include "weight-dropoff.h"
#include "mission.h"
#include "lift-motor.h"
#include "smart-servo.h"
#include "motion-controller.h"
#include "odometry.h"

#define UNLOAD_DELAY_MS 5000
#define DROPOFF_DRIVE_SPEED 0.07f

static weight_dropoff_state_t current_state = DROPOFF_STATUS_IDLE;
static unsigned long unload_start_time = 0;


void weight_dropoff_state_update()
{
    switch (current_state)
    {
    case DROPOFF_STATUS_IDLE:
        if (mission_get_state() == MISSION_COMPLETE)
        {
            //at home base
            lifter_lower();
            current_state = DROPOFF_STATUS_LOWER;
        }
        break;

    case DROPOFF_STATUS_LOWER:
        if (is_lifter_reached_target())
        {
            set_back_servo_up();
            motion_controller_override_drive(DROPOFF_DRIVE_SPEED, 0.0f);
            unload_start_time = millis();
            current_state = DROPOFF_STATUS_UNLOAD;
        }
        break;

    case DROPOFF_STATUS_UNLOAD:
        if (millis() - unload_start_time > UNLOAD_DELAY_MS)
        {
            motion_controller_override_drive(0.0f, 0.0f);
            set_back_servo_down();
            mission_report_dropoff_complete(true);
            current_state = DROPOFF_STATUS_IDLE;
        }
        break;
    }
}
