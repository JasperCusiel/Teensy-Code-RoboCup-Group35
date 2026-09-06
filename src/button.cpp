//
// Created by Jasper Cusiel on 03/08/2026.
//
#include "button.h"
#include <arduino.h>

#define DEBOUNCE_TIME_MS 10

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
