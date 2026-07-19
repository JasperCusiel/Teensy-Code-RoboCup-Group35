//
// Created by Jasper Cusiel on 20/07/2026.
//

#include "display.h"
#include <Arduino.h>
#include <U8x8lib.h>

static U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C display(U8X8_PIN_NONE);
#define MAX_LINES 8

static char lines[MAX_LINES][17];
static uint8_t lineCount = 0;

void display_init() {
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.clear();
}

void draw() {
  display.clear();
  for (uint8_t i = 0; i < lineCount; i++) {
    display.drawString(0, i, lines[i]);
  }
}

void display_log(const char* msg) {
  if (lineCount >= MAX_LINES) {
    for (int i = 0; i < MAX_LINES - 1; i++) {
      strcpy(lines[i], lines[i + 1]);
    }
    strncpy(lines[MAX_LINES - 1], msg, 16);
    lines[MAX_LINES - 1][16] = '\0';
  } else {
    strncpy(lines[lineCount], msg, 16);
    lines[lineCount][16] = '\0';
    lineCount++;
  }

  draw();
}

void display_log_status(const char* name, bool ok) {
  char buffer[17];
  snprintf(buffer, sizeof(buffer), "%-10s %s", name, ok ? "OK" : "FAIL");
  display_log(buffer);
  draw();
}

