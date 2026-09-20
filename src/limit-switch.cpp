//
// Created by Jasper Cusiel on 21/07/2026.
//

#include "limit-switch.h"
#include <Wire.h>   // SparkFunSX1509.h does not include this for some reason?
#include <SparkFunSX1509.h>


#define DEBOUNCE_TIME_MS 10 // How long to debounce switches

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address
SX1509 digitalIO; // Create an SX1509 object to be used throughout


bool limit_switches_init()
{
    // Initialize IO expander and set pinmodes for limitswitches
    if (!digitalIO.begin(SX1509_ADDRESS))
    {
        Serial.println("Failed to communicate with SX1509.");
        return false;
    }
    digitalIO.pinMode(SX1509_AIO0, INPUT);
    digitalIO.pinMode(SX1509_AIO1, INPUT);
    // heading encoder
    digitalIO.pinMode(SX1509_AIO12, INPUT);
    digitalIO.pinMode(SX1509_AIO13, INPUT);
    digitalIO.pinMode(SX1509_AIO14, INPUT);
    digitalIO.pinMode(SX1509_AIO15, INPUT);

    return true;
}

bool read_limit_switch(uint8_t pin)
{
    // Read and debounce limit switch
    static uint32_t lastChange = 0;
    static bool lastState = HIGH;

    bool current = !digitalIO.digitalRead(pin);

    if (current != lastState)
    {
        lastChange = millis();
        lastState = current;
    }

    if (millis() - lastChange > DEBOUNCE_TIME_MS)
    {
        return current;
    }

    return lastState;
}

bool read_io_pin(uint8_t pin)
{
    return digitalIO.digitalRead(pin);
}
