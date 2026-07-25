//
// Created by Jasper Cusiel on 21/07/2026.
//

#include <limit-switch.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

#define DEBOUNCE_TIME_MS 20

const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 digitalIO; // Create an SX1509 object to be used throughout

// SX1509 Pins:
const byte SX1509_AIO0 = 0;
const byte SX1509_AIO1 = 1;
const byte SX1509_AIO2 = 2;
const byte SX1509_AIO3 = 3;
const byte SX1509_AIO4 = 4;
const byte SX1509_AIO5 = 5;
const byte SX1509_AIO6 = 6;
const byte SX1509_AIO7 = 7;
const byte SX1509_AIO8 = 8;
const byte SX1509_AIO9 = 9;
const byte SX1509_AIO10 = 10;
const byte SX1509_AIO11 = 11;
const byte SX1509_AIO12 = 12;
const byte SX1509_AIO13 = 13;
const byte SX1509_AIO14 = 14;
const byte SX1509_AIO15 = 15;


bool limit_switches_init() {
  if (!digitalIO.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    return false;
  }
  Serial.println("Limit switches init");
  digitalIO.pinMode(SX1509_AIO0, INPUT);
  // digitalIO.pinMode(SX1509_AIO1, INPUT);

  return true;
}

bool readLimitSwitch(uint8_t pin) {
  static uint32_t lastChange = 0;
  static bool lastState = HIGH;

  bool current = !digitalIO.digitalRead(pin);

  if (current != lastState) {
    lastChange = millis();
    lastState = current;
  }

  if (millis() - lastChange > DEBOUNCE_TIME_MS) {
    return current;
  }

  return lastState;
}