//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_DISPLAY_H
#define ROBOCUP_DISPLAY_H
#include <stdint.h>

enum Page {
  PAGE_MENU,
  PAGE_VFH,
  PAGE_DEBUG,
  PAGE_ODOM,
  PAGE_8X8_TOF,
  PAGE_MAP,
  PAGE_BOOT_STATUS
};

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
void draw_boot_status();
void display_set_page(Page new_page);
void draw_map();
void draw_robot_arrow(int cx, int cy, float angle, uint8_t length);


#endif // ROBOCUP_DISPLAY_H
