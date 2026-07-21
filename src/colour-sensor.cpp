//
// Created by Jasper Cusiel on 21/07/2026.
//
#include <colour-sensor.h>
#include <Adafruit_TCS34725.h>

#define I2C_BUS &Wire1
#define I2C_ADDRESS 0x29

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

bool colour_sensor_init() {
  for (int i = 0; i < 20; i++) {  // 2 seconds max
    if (tcs.begin(I2C_ADDRESS, I2C_BUS) == true) {
      return true;
    }
    delay(100);
  }
  return false;
}