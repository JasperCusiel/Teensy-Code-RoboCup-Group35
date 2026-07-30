//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_DISPLAY_H
#define ROBOCUP_DISPLAY_H
#include <stdint.h>

void display_init();
void draw();
void display_log(const char* msg);
void display_log_status(const char* name, bool ok);
void draw_vfh(const float* histogram);
void draw_debug();
void draw_odometry();
bool read_button(uint8_t pin);
void update_input();
void draw_menu();

#endif // ROBOCUP_DISPLAY_H
