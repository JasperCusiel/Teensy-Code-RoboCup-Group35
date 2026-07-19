#include "Bitcraze_PMW3901.h"
#include "U8x8lib.h"
#include "Wire.h"
#include <Arduino.h>
#include "ToF-Sensors.h"
#include "display.h"

void scanI2C() {
  display_log("Scanning I2C bus...");

  uint8_t count = 0;
  char buffer[18];

  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

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
  Serial1.begin(115200);
  Wire.begin();
  Wire1.begin();

  display_init();
  display_log("Booting...");


  tof_init();
  display_log_status("TOF ARRAY", tof_init());
  // scanI2C();

  // Calibration
  //get_ToFCalibration();

}



void loop() {
  get_tof_reading();
}
