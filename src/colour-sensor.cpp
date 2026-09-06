//
// Created by Jasper Cusiel on 21/07/2026.
//
#include <colour-sensor.h>
#include <Adafruit_TCS34725.h>

#define I2C_BUS &Wire1
#define I2C_ADDRESS 0x29


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

color_t base = {0, 0, 0, 0, COLOR_UNDEFINED};

bool colour_sensor_init() {
  for (int i = 0; i < 20; i++) {  // 2 seconds max
    if (tcs.begin(I2C_ADDRESS, I2C_BUS) == true) {
      // Get base color
      update_base_color();
      return true;
    }
    delay(100);
  }


  return false;
}

void update_base_color() {
  tcs.getRawData(&base.red, &base.green, &base.blue, &base.clear);
  if (base.green > base.blue) {
    base.base_color = COLOR_GREEN;
  } else {
    base.base_color = COLOR_BLUE;
  }
}

base_color_t get_base_color() {
  return base.base_color;
}