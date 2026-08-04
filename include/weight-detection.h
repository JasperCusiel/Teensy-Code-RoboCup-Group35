//
// Created by Jasper Cusiel on 03/08/2026.
//

#ifndef ROBOCUP_WEIGHT_DETECTION_H
#define ROBOCUP_WEIGHT_DETECTION_H
#include "U8g2lib.h"
bool weight_detection_init();
void drawToF_dithered_fast(U8G2 &u8g2,
                           uint16_t d_max,
                           int x0, int y0);
void draw_depth_data(U8G2 &u8g2);
void fill_calibration_matrix();

#endif // ROBOCUP_WEIGHT_DETECTION_H
