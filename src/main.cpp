#include "Wire.h"
#include <Arduino.h>
#include "ToF-Sensors.h"
#include "display.h"
#include "smart-servo.h"
#include <imu.h>
#include <optical-flow.h>
#include <colour-sensor.h>
#include <inductive-sensor.h>
#include <lift-motor.h>
#include <limit-switch.h>
#include <vfh.h>

#define GO_BTN A6


lidar_scan scan;

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

  // Start and check sensors
  display_log_status("TOF ARRAY", tof_init());
  display_log_status("SERVOS", smart_servo_init());
  display_log_status("IMU", imu_init());
  display_log_status("OPT FLOW", optical_flow_init());
  display_log_status("COLOUR", colour_sensor_init());
  display_log_status("INDUCTIVE", inductive_sensor_init());
  display_log_status("LIMIT SW's", limit_switches_init());
  //display_log_status("LIFTER SERVO", lifter_motor_init());

  vfh_init();
  calculate_sector_indices(&scan);


  // scanI2C();

  // Calibration
  //get_ToFCalibration();

  // Wait for GO button to be pushed to start program
  pinMode(GO_BTN, INPUT);
  while (digitalRead(GO_BTN) == LOW) {
    delay(50);
  }
  Serial.println("Pushed");
  while (digitalRead(GO_BTN) == HIGH) {
    delay(50);
  }
  Serial.println("Released");
  start_vfh();
}



void loop() {
  get_tof_reading(&scan);
  float target_angle = 0.0f;
  set_target_angle(target_angle);
  compute_vfh(&scan);

  // Serial.print("Steer Angle:");
  // if (isnan(steering_angle)) {
  //   Serial.println("NAN");
  // } else {
  //   steering_angle = steering_angle * (180 / Pi);
  //   Serial.println(steering_angle);
  // }
  draw();


}
