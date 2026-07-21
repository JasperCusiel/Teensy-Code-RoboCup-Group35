//
// Created by Jasper Cusiel on 21/07/2026.
//
#include <inductive-sensor.h>

#include <core_pins.h>

#define SENSE_PIN A0

bool inductive_sensor_init() {
  pinMode(SENSE_PIN, INPUT);
  if (digitalRead(SENSE_PIN)) {
    return true;
  }
  return false;
}
