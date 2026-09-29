#include "ToF-Sensors.h"
#include "Wire.h"
#include "autonomy.h"
#include "display.h"
#include "drivetrain.h"
#include "mapping.h"
#include "mission.h"
#include "motion-controller.h"
#include "odometry.h"
#include "sensors.h"
#include "smart-servo.h"
#include "status-leds.h"
#include "telemetry.h"
#include "weight-detection.h"
#include "weight-pickup.h"
#include <Arduino.h>
#include <colour-sensor.h>
#include <imu.h>
#include <inductive-sensor.h>
#include <lift-motor.h>
#include <limit-switch.h>
#include <optical-flow.h>
#include <vfh.h>
#include "heading-encoder.h"
#include "IR-reflective.h"
#include "weight-dropoff.h"

#include "button.h"
#include "scheduler.h"
#define HZ_TO_US(x) (1000000UL / (x)) // convert hz to micro seconds
#define NUM_TASKS (sizeof(tasks) / sizeof(tasks[0]))

#define GO_BTN A9

// All tasks for scheduler to run
task_t tasks[] = {
    {imu_task, HZ_TO_US(95), 0},
    {odometry_update, HZ_TO_US(95), 0},
    {autonomy_motion_task, HZ_TO_US(95), 0},
    {drivetrain_update, HZ_TO_US(100), 0},
    //{display_draw, HZ_TO_US(4), 0},
    {update_input, HZ_TO_US(5), 0},
    {get_tof_reading, HZ_TO_US(1000), 0},
    {mapping_task, HZ_TO_US(6), 0},
    {autonomy_task, HZ_TO_US(20), 0},
    {telemetry_map_task, HZ_TO_US(6), 0},
    {lifter_motor_update, HZ_TO_US(100), 0},
    {weight_pickup_state_update, HZ_TO_US(10), 0},
    {status_leds_task, HZ_TO_US(1), 0},
    {ir_reflective_update, HZ_TO_US(5), 0},
    {weight_dropoff_state_update, HZ_TO_US(10), 0},
    {weight_detection_task, HZ_TO_US(20), 0},
    {smart_servo_monitor_task, HZ_TO_US(1), 0},
    //{drivetrain_debug_task, HZ_TO_US(2), 0}
};


void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire1.begin();
    display_init();
    sensors_init();
    vfh_init();
    odometry_init();
    mapping_init();
    telemetry_init();
    drivetrain_init();
    autonomy_init();
    status_leds_init();
    Serial.print("Base Colour: ");
    if (get_base_color() == COLOR_GREEN)
    {
        Serial.println("GREEN");
    }
    else
    {
        Serial.println("BLUE");
    }

    Serial.println("Push GO BTN to start");
    pinMode(GO_BTN, INPUT);

    // Wait for a press.
    while (!read_button(GO_BTN))
    {
        delay(1);
    }

    // Wait for release before starting.
    while (read_button(GO_BTN))
    {
        delay(1);
    }

    display_set_page(PAGE_8X8_TOF);
    Serial.println("Start");
    mission_start();
    scheduler_init(tasks, NUM_TASKS);
}


void loop()
{
    scheduler_run(tasks, NUM_TASKS);
}
