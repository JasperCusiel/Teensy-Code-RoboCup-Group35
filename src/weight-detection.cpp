//
// Created by Jasper Cusiel on 03/08/2026.
//
#include "weight-detection.h"
#include "DFRobot_MatrixLidar.h"
#include "button.h"


#define TOF_ARRAY_ADDRESS 0x33

DFRobot_MatrixLidar_I2C tof(TOF_ARRAY_ADDRESS, &Wire1);
uint16_t buf[64];
uint32_t calibration[64];


bool weight_detection_init() {
  // Check sensor starts and set to 8x8 mode
  if (tof.begin() == 0 && tof.setRangingMode(eMatrix_8X8) == 0) {
    return true;
  }
  return false;
}

void drawToF_dithered_fast(U8G2 &u8g2,
                           uint16_t d_max,
                           int x0, int y0)
{
  // 4x4 Bayer matrix (0-15)
  static const uint8_t bayer[4][4] = {
    { 0,  8,  2, 10},
    {12,  4, 14,  6},
    { 3, 11,  1,  9},
    {15,  7, 13,  5}
  };

  const int cell_size = 8;

  // Scale d_max difference to 0-15 brightness
  uint32_t scale = (15UL << 12) / d_max;

  uint16_t max_value = 0;

  for (int cy = 0; cy < 8; cy++) {
    for (int cx = 0; cx < 8; cx++) {

      int index = cy * 8 + cx;

      // Remove calibrated floor
      int16_t d = (int16_t)calibration[index] -
                  (int16_t)buf[index];

      // Ignore anything further than the floor
      if (d < 0)
        d = 0;

      if (d > d_max)
        d = d_max;

      if (d > max_value)
        max_value = d;


      // Convert to brightness 0-15
      uint32_t norm = (uint32_t)d * scale;
      uint8_t level = norm >> 12;


      int base_x = x0 + cx * cell_size;
      int base_y = y0 + cy * cell_size;


      // Draw 8x8 dithered block
      for (int py = 0; py < cell_size; py++) {

        int sy = base_y + py;
        const uint8_t *row = bayer[py & 3];

        for (int px = 0; px < cell_size; px++) {

          if (level > row[px & 3]) {
            u8g2.drawPixel(base_x + px, sy);
          }
        }
      }
    }
  }


  char buffer[20];
  snprintf(buffer, sizeof(buffer), "Max:%hu", max_value);
  u8g2.drawStr(70, 20, buffer);


  // Calibration button
  if (read_button(A9) == LOW) {
    u8g2.drawStr(70, 10, "Calibrating...");
    fill_calibration_matrix();
  }
}


void draw_depth_data(U8G2 &u8g2) {
  tof.getAllData(buf);
  drawToF_dithered_fast(u8g2, 350, 0, 0);
}

void fill_calibration_matrix() {
  // Reset calibration data

  for (size_t j = 0; j < 64; j++) {
    calibration[j] = 0;
  }

  for (size_t i = 0; i < 10; i++) {
    tof.getAllData(buf);
    // Add data to calibration buffer
    for (size_t j = 0; j < 64; j++) {
      calibration[j] += buf[j];
    }
    delay(50); // Wait for new frame

  }
  // Average data
  for (size_t j = 0; j < 64; j++) {
    calibration[j] = calibration[j] / 10;
  }
}