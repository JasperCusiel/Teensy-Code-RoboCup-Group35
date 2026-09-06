//
// Created by Jasper Cusiel on 05/08/2026.
//
#include "sensors.h"

#include "smart-servo.h"
#include "imu.h"
#include "optical-flow.h"
#include "colour-sensor.h"
#include "inductive-sensor.h"
#include "lift-motor.h"
#include "limit-switch.h"
#include "weight-detection.h"
#include "ToF-Sensors.h"
#include "lift-motor.h"
#include "display.h"

static bool boot_okay = true;
static const char *failed_sensor = nullptr;


sensor_t sensors[] = {
  {false, tof_init, "TOF ARRAY"},
  {false, smart_servo_init, "SERVOS"},
  {false, imu_init, "IMU"},
  {false, optical_flow_init, "OPT FLOW"},
  {false, colour_sensor_init, "COLOUR"},
  {false, inductive_sensor_init, "INDUCTIVE"},
  {false, limit_switches_init, "LIMIT SW'S"},
  {false, weight_detection_init, "8X8 TOF"},
  {false,  colour_sensor_init, "COLOR SENS"}
// {false, lifter_motor_init, "LIFTER SERVOS"}
};

void sensors_init() {
  boot_okay = true;
  failed_sensor = nullptr;

  for (size_t i = 0; i < NUM_SENSORS; i++) {

    bool status = sensors[i].init_func();
    sensors[i].boot_okay = status;

    if (!status && boot_okay) {
      boot_okay = false;
      failed_sensor = sensors[i].name;
    }

    display_log_status(sensors[i].name, status);
  }
  display_set_page(PAGE_BOOT_STATUS);
  draw();
  delay(2000);
  display_set_page(PAGE_MENU);
}

bool sensors_boot_okay() {
  return boot_okay;
}


const char* sensors_failed_sensor() {
  return failed_sensor;
}
