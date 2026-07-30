//
// Created by Jasper Cusiel on 20/07/2026.
//

#include "display.h"

#include "ToF-Sensors.h"
#include "lidar-config.h"
#include "vfh.h"
#include "odometry.h"



#include <Arduino.h>
#include <Encoder.h>
#include "U8g2lib.h"

#define ENCODER_A A13
#define ENCODER_B A12
#define SW_PIN A11

Encoder  input_encoder(ENCODER_A, ENCODER_B);

enum Page {
  PAGE_MENU,
  PAGE_VFH,
  PAGE_DEBUG,
  PAGE_ODOM
};

const char *menuItems[] = {
  "VFH",
  "Debug",
  "Odometry"
};
#define MENU_ITEMS_COUNT 3
#define DEBOUNCE_TIME_MS 20

Page currentPage = PAGE_DEBUG;
int menuIndex = 0;
int32_t last_encoder_count = 0;

#define HISTOGRAM_X 64
#define HISTOGRAM_Y 63

// static U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C display(U8X8_PIN_NONE);
static U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
#define MAX_LINES 7

static char lines[MAX_LINES][17];
static uint8_t lineCount = 0;

bool read_button(uint8_t pin) {
  static uint32_t lastChange = 0;
  static bool lastState = HIGH;

  bool current = !digitalRead(pin);

  if (current != lastState) {
    lastChange = millis();
    lastState = current;
  }

  if (millis() - lastChange > DEBOUNCE_TIME_MS) {
    return current;
  }

  return lastState;
}

void update_input() {
  int32_t delta = input_encoder.readAndReset() / 2;

  if (currentPage == PAGE_MENU) {

    if (delta != 0) {
      menuIndex += delta;

      if (menuIndex < 0)
        menuIndex = 0;

      if (menuIndex >= MENU_ITEMS_COUNT)
        menuIndex = MENU_ITEMS_COUNT - 1;
    }
    if (read_button(SW_PIN)) {
      currentPage = (Page)(menuIndex + 1);
    }
  }
  else {

    if (read_button(SW_PIN)) {
      currentPage = PAGE_MENU;
    }
  }
}

void draw_menu() {

  for (int i = 0; i < MENU_ITEMS_COUNT; i++) {
    int y = (i + 1) * 12;

    if (i == menuIndex) {
      display.drawBox(0, y - 10, 128, 12);
      display.setDrawColor(0);
      display.drawStr(2, y, menuItems[i]);
      display.setDrawColor(1);
    } else {
      display.drawStr(2, y, menuItems[i]);
    }
  }
}

void display_init() {
  display.begin();
  display_log("BOOT OK");
  currentPage = PAGE_DEBUG;

  pinMode(SW_PIN, INPUT_PULLUP);
  // long oldPosition  = -999;
  // while (true) {
  //   // Read the current position of the encoder
  //   long newPosition = input_encoder.read();
  //
  //   // Only print if the value has changed
  //   if (newPosition != oldPosition) {
  //     oldPosition = newPosition;
  //     Serial.print("Position: ");
  //     Serial.println(newPosition);
  //   }
  //
  //   // Read the push-button switch state (LOW means pressed)
  //   if (digitalRead(SW_PIN) == LOW) {
  //     Serial.println("Button Pressed!");
  //     delay(250); // Simple debounce delay
  //   }
  // }

}

void draw() {
  display.firstPage();

  do {
    display.setFont(u8g2_font_6x12_tr);
    switch (currentPage) {
    case PAGE_MENU:  draw_menu(); break;
    case PAGE_VFH:     draw_vfh(get_histogram()); break;
    case PAGE_ODOM: draw_odometry(); break;
    case PAGE_DEBUG:   draw_debug(); break;
    }

  } while (display.nextPage());
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

void draw_debug() {
  for (uint8_t i = 0; i < lineCount; i++) {
    display.drawStr(0, 10 * i, lines[i]);
  }
}

int angle_to_u8g2(float angle)
{
  int a = 64 - (angle * 256.0f / (2.0f * PI));

  // wrap into 0-255
  while (a < 0)   a += 256;
  while (a >= 256) a -= 256;

  return a;
}

void draw_angle_arrow(float angle, uint8_t radius)
{
  int cx = HISTOGRAM_X;
  int cy = HISTOGRAM_Y;


  int x0 = cx + sinf(angle) * radius;
  int y0 = cy - cosf(angle) * radius;


  uint8_t arrow_length = 8;

  int x1 = cx + sinf(angle) * (radius + arrow_length);
  int y1 = cy - cosf(angle) * (radius + arrow_length);


  display.drawLine(x0, y0, x1, y1);


  float head_angle = 0.5f;
  uint8_t head_length = 4;

  int xa = x1 - sinf(angle + head_angle) * head_length;
  int ya = y1 + cosf(angle + head_angle) * head_length;

  int xb = x1 - sinf(angle - head_angle) * head_length;
  int yb = y1 + cosf(angle - head_angle) * head_length;

  display.drawLine(x1, y1, xa, ya);
  display.drawLine(x1, y1, xb, yb);
}

void draw_vfh(const float* histogram) {

  uint8_t min_radius = 10;
  uint8_t max_radius = 40;

  for (int i = 0; i < NUM_SECTORS; i++) {

    float angle_start = FOV_MAX - i * SECTOR_WIDTH;
    float angle_end   = angle_start - SECTOR_WIDTH;

    uint8_t start = angle_to_u8g2(angle_start);
    uint8_t end   = angle_to_u8g2(angle_end);


    float h = constrain(histogram[i], 0.0f, 1.0f);

    uint8_t radius = min_radius + h * (max_radius - min_radius);

    display.drawArc(
      HISTOGRAM_X,
      HISTOGRAM_Y,
      radius,
      start,
      end
    );
    display.drawArc(HISTOGRAM_X, HISTOGRAM_Y, 45, angle_to_u8g2(60.0f * PI / 180.0f), angle_to_u8g2(-60.0f * PI / 180.0f));
    draw_angle_arrow(get_target_angle(), 45);
    draw_angle_arrow(get_steering_angle(), 45+8);
  }
}


void draw_odometry() {
  // Title
  display.drawStr(30, 0, "Odometry");

  float x, y, theta;
  get_ekf_pose(&x, &y, &theta);

  float gyro_z, heading, vx, vy;
  get_sensor_data(&gyro_z, &heading, &vx, &vy);

  // display.drawStr(0, 2, "EKF Pose");

  char buf[20];
  // Draw X Y Theta
  snprintf(buf, sizeof(buf), "X:%5.2f", x);
  display.drawStr(0, 12, buf);

  snprintf(buf, sizeof(buf), "Y:%5.2f", y);
  display.drawStr(0, 24, buf);

  snprintf(buf, sizeof(buf), "Theta:%5.2f", degrees(theta));
  display.drawStr(0, 36, buf);


  // Raw sensor data
  snprintf(buf, sizeof(buf), "Gyro Z:%5.2f", gyro_z);
  display.drawStr(0, 48, buf);

  snprintf(buf, sizeof(buf), "HDG:%5.2f", degrees(heading));
  display.drawStr(64, 12, buf);

  snprintf(buf, sizeof(buf), "vX:%5.2f", vx);
  display.drawStr(0, 60, buf);

  snprintf(buf, sizeof(buf), "vY:%5.2f", vx);
  display.drawStr(64, 60, buf);

}

