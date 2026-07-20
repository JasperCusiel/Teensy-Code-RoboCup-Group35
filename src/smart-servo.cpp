//
// Created by Jasper Cusiel on 20/07/2026.
//

#include <smart-servo.h>
#include <HerkulexServo.h>

#define SERIAL_BUS Serial2
#define SERIAL_BAUD 115200 // max is 0.67 MBPS
#define SERVO_ID_A 02
#define SERVO_ID_B 03

HerkulexServoBus herkulex_bus(SERIAL_BUS);
HerkulexServo    servo_a(herkulex_bus, SERVO_ID_A);
HerkulexServo    servo_b(herkulex_bus, SERVO_ID_B);

HerkulexServo* servos[] = {&servo_a, &servo_b};
uint8_t servo_id[] = {SERVO_ID_A, SERVO_ID_B};

HerkulexStatusError servo_error;
HerkulexStatusDetail detail;

bool smart_servo_init() {
  SERIAL_BUS.begin(SERIAL_BAUD);

  uint8_t num_errors = 0;

  for (size_t i = 0; i < 2; i++) {
    HerkulexPacket resp;
    if (bool success = herkulex_bus.sendPacketAndReadResponse(
            resp, servo_id[i], HerkulexCommand::Stat);
        !success) {
      Serial.printf("Servo %d no response\n", i);
      num_errors++;
      continue;
    }
    servos[i]->getStatus(servo_error, detail);

    Serial.printf("Servo %d error: 0x%02X detail: 0x%02X\n",
                  i,
                  (uint8_t)servo_error,
                  (uint8_t)detail);

    servos[i]->setLedColor(HerkulexLed::Green);
    servos[i]->setTorqueOn();

    if (servo_error != HerkulexStatusError::None) {
      num_errors++;
    }
  }

  return num_errors == 0;
}
