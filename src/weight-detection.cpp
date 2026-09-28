//
// Created by Jasper Cusiel on 03/08/2026.
//
#include "weight-detection.h"
#include "DFRobot_MatrixLidar.h"
#include "button.h"


#define TOF_ARRAY_ADDRESS 0x33
#define WEIGHT_MIN_DELTA 25
#define WEIGHT_MAX_DELTA 150
#define WEIGHT_MAX_ACTIVE_CELLS 32
#define WALL_FRONT_CONE_RAD (35.0f * Pi / 180.0f)
#define WALL_MIN_RANGE_M 0.05f
#define WALL_MAX_RANGE_M 0.55f
#define WALL_MAX_RANGE_SPREAD_M 0.12f
#define WALL_MIN_ADJACENT_POINTS 4
#define WEIGHT_DETECTED_CYCLES 5


DFRobot_MatrixLidar_I2C tof(TOF_ARRAY_ADDRESS, &Wire1);
uint16_t buf[64];
uint32_t calibration[64];
bool active[64];
bool weight_detected = false;

bool wall_detected = false;

enum WeightType
{
    WEIGHT_NONE,
    WEIGHT_2X2,
    WEIGHT_2X3
};

WeightType weight_type = WEIGHT_NONE;


static bool tof_array_sees_wall();



bool weight_detection_init() {
  // Check sensor starts and set to 8x8 mode
  if (tof.begin() == 0 && tof.setRangingMode(eMatrix_8X8) == 0) {
    return true;
  }
  return false;
}

<<<<<<< Updated upstream
void drawToF_dithered_fast(U8G2 &u8g2,
=======
void weight_detection_task()
{
    static uint32_t last_sample_ms = 0;
    static uint32_t sum[64] = {0};
    static uint8_t sample_count = 0;

    const uint32_t now = millis();

    // Sample every 20 ms without blocking.
    if ((now - last_sample_ms) < 20U)
    {
        return;
    }
    last_sample_ms = now;

    uint16_t temp[64];
    tof.getAllData(temp);

    for (int i = 0; i < 64; i++)
    {
        sum[i] += temp[i];
    }

    sample_count++;

    // Wait until we have 8 averaged samples before processing.
    if (sample_count < 8)
    {
        return;
    }

    for (int i = 0; i < 64; i++)
    {
        buf[i] = sum[i] / 8U;
        sum[i] = 0;
    }
    sample_count = 0;

    int active_count = 0;

    for (int i = 0; i < 64; i++)
    {
        const int16_t delta = (int16_t)calibration[i] - (int16_t)buf[i];
        active[i] = (delta > WEIGHT_MIN_DELTA && delta < WEIGHT_MAX_DELTA);

        if (active[i])
        {
            active_count++;
        }
    }

    if (active_count > WEIGHT_MAX_ACTIVE_CELLS)
    {
        for (int i = 0; i < 64; i++)
        {
            active[i] = false;
        }
    }

    filter();
    wall_detected = tof_array_sees_wall();
    weight_detected = !wall_detected && detect_weight();

    if (weight_detected)
    {
        mission_report_weight_detected();
    }
}

void drawToF_dithered_fast(U8G2& u8g2,

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

    if (weight_detected)
    {
        switch (weight_type){
            case WEIGHT_2X2:
            u8g2.drawStr(70, 10, "2X2");
            break;

            case WEIGHT_2X3:
            u8g2.drawStr(70, 10, "2X3");
            break;

            default:
            u8g2.drawStr(70, 10, "WEIGHT");
            break;
        }
    }
    else if (wall_detected)
    {
        u8g2.drawStr(70, 10, "WALL");
    }


  uint16_t max_value = 0;

  for (int i = 0; i <64; i ++){

  int16_t d = (int16_t)calibration[i] - (int16_t)buf[i];

    active[i] = (d >0 && d < d_max);
  }
  filter();
  weight_detected = detect_weight();

  if (weight_detected){
    u8g2.drawStr(70,10, "WEIGHT!");
  }

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
        d = 0;

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
  }



void filter() //filter out random pixels
{
    bool filtered[64] = {false};

    for (int cy = 0; cy < 8; cy++) {
        for (int cx = 0; cx < 8; cx++) {

            int index = cy * 8 + cx;

            if (!active[index]){
              continue;
            }
          
            int neighbours = 0;
            if (cx > 0 && active[index - 1]){
                neighbours++;
            }
            if (cx < 7 && active[index + 1]){
                neighbours++;
            }
            if (cy > 0 && active[index - 8]){
                neighbours++;
            }

            if (cy < 7 && active[index + 8]){
                neighbours++;
            }

            if (neighbours >= 2){
                filtered[index] = true;
            }
        }
    }

    // Copy filtered result back
    for (int i = 0; i < 64; i++) {
        active[i] = filtered[i];
    }
}

bool detect_weight(){ //by finding a stack of 3 pixels in a coloumn (unsure)
  for (int y = 0; y < 6; y++ ){
    for(int x = 0; x < 8; x++){

      int i = y * 8 + x;
      if(active[i] && active[i + 8] && active[i+16]){
        return true;
      }
    }
  }

  return false;
}


void draw_depth_data(U8G2 &u8g2) {
  // tof.getAllData(buf);

  uint32_t sum[64] = {0};
  uint16_t temp[64];

for(int n = 0; n < 8; n++) {
    tof.getAllData(temp);

    for(int i = 0; i < 64; i++) {
        sum[i] += temp[i];
    }

    delay(20);


static bool is_active_cell(int x, int y)
{
    if (x < 0 || x >= 8 || y < 0 || y >= 8)
    {
        return false;
    }

    return active[y * 8 + x];
}

static bool is_exact_2x2(int x, int y)
{
    if (x < 0 || y < 0 || x + 1 >= 8 || y + 1 >= 8)
    {
        return false;
    }

    // Require the exact 2x2 block
    if (!is_active_cell(x, y) || !is_active_cell(x + 1, y) ||
        !is_active_cell(x, y + 1) || !is_active_cell(x + 1, y + 1))
    {
        return false;
    }

    // Block larger patterns like 2x3, 3x2, 2x4 etc.
    if (is_active_cell(x - 1, y) || is_active_cell(x + 2, y) ||
        is_active_cell(x - 1, y + 1) || is_active_cell(x + 2, y + 1) ||
        is_active_cell(x, y - 1) || is_active_cell(x + 1, y - 1) ||
        is_active_cell(x, y + 2) || is_active_cell(x + 1, y + 2))
    {
        return false;
    }

    return true;
}

static bool is_exact_2x3(int x, int y)
{
    if (x < 0 || y < 0 || x + 1 >= 8 || y + 2 >= 8)
    {
        return false;
    }

    // Require the exact 2x3 block
    if (!is_active_cell(x, y) || !is_active_cell(x + 1, y) ||
        !is_active_cell(x, y + 1) || !is_active_cell(x + 1, y + 1) ||
        !is_active_cell(x, y + 2) || !is_active_cell(x + 1, y + 2))
    {
        return false;
    }

    // Reject larger patterns like 2x4, 3x2, 3x3
    if (is_active_cell(x - 1, y) || is_active_cell(x + 2, y) ||
        is_active_cell(x - 1, y + 1) || is_active_cell(x + 2, y + 1) ||
        is_active_cell(x - 1, y + 2) || is_active_cell(x + 2, y + 2) ||
        is_active_cell(x, y - 1) || is_active_cell(x + 1, y - 1) ||
        is_active_cell(x, y + 3) || is_active_cell(x + 1, y + 3))
    {
        return false;
    }

    return true;
}

bool detect_weight()
{
    for (int y = 1; y < 7; y++)
    {
        for (int x = 1; x < 7; x++)
        {
            if (is_exact_2x2(x, y))
            {
                weight_type = WEIGHT_2X2;
                return true;
            }

            if (is_exact_2x3(x, y))
            {
                weight_type = WEIGHT_2X3;
                return true;
            }
        }
    }

    weight_type = WEIGHT_NONE;
    return false;

}

for(int i = 0; i < 64; i++) {
    buf[i] = sum[i] / 8;
}


  drawToF_dithered_fast(u8g2,150,0, 0);
  if (read_button(A9) == LOW) {
    u8g2.drawStr(70, 10, "calibrating...");
    fill_calibration_matrix();

}
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

    drawToF_dithered_fast(u8g2, 200, 0, 0);
    if (read_button(A9) == LOW)
    {
        u8g2.drawStr(70, 10, "calibrating...");
        fill_calibration_matrix();
    }

}



