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
#include "telemetry.h"
#include "weight-detection.h"
#include <Arduino.h>
#include <FastLED.h>
#include <colour-sensor.h>
#include <imu.h>
#include <inductive-sensor.h>
#include <lift-motor.h>
#include <limit-switch.h>
#include <optical-flow.h>
#include <vfh.h>

#include "scheduler.h"
#define HZ_TO_US(x) (1000000UL / (x)) // convert hz to micro seconds
#define NUM_TASKS (sizeof(tasks) / sizeof(tasks[0]))


#define GO_BTN A9

#define NUM_LEDS 32
#define DATA_PIN A12
CRGB leds[NUM_LEDS];


uint32_t last_time_1 = micros();

float state[3];

task_t tasks[] = {
  { imu_get_reading, HZ_TO_US(95), 0 },
  { odometry_update,  HZ_TO_US(95), 0 },
  { autonomy_motion_task, HZ_TO_US(95), 0 },
  { draw,   HZ_TO_US(5),  0 },
  { update_input,   HZ_TO_US(5),  0 },
  { get_tof_reading, HZ_TO_US(6),  0 },
  { mapping_task, HZ_TO_US(6),  0 },
  { autonomy_task, HZ_TO_US(20), 0 },
  { telemetry_map_task, HZ_TO_US(6), 0 }
};


void scanI2C() {
  display_log("Scanning I2C bus...");

  uint8_t count = 0;
  char buffer[18];

  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire1.beginTransmission(addr);
    uint8_t error = Wire1.endTransmission();

    if (error == 0) {
      snprintf(buffer, sizeof(buffer), "0x%02X found", addr);
      display_log(buffer);
      delay(150);
      count++;
    }
  }

  if (count == 0)
    display_log("No I2C devices");
  else {
    snprintf(buffer, sizeof(buffer), "%d devices found", count);
    display_log(buffer);
  }
}



void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();

  display_init();
  sensors_init();
  vfh_init();
  colour_sensor_init();
  odometry_init();
  mapping_init();
  telemetry_init();
  drivetrain_init();
  // motion_controller_set_output_callback(set_motor_speeds);
  // set_motor_speeds(0.9f, 0.9f);
  autonomy_init();
  Serial.print("Base Colour: ");
  if (get_base_color() == COLOR_GREEN) {
    Serial.println("GREEN");
  } else {
    Serial.println("BLUE");
  }

  // scanI2C();

  // Calibration
  //get_ToFCalibration();

  FastLED.addLeds<WS2812,DATA_PIN,RGB>(leds,NUM_LEDS);
  FastLED.setBrightness(128);
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();

  Serial.println("Push GO BTN to start");
  // Wait for GO button to be pushed to start program
  pinMode(GO_BTN, INPUT);

  while (read_button(GO_BTN)) {
    delay(1);
  }
  Serial.println("Start");
  mission_start();
  scheduler_init(tasks, NUM_TASKS);
}



void loop() {
  scheduler_run(tasks, NUM_TASKS);
}
