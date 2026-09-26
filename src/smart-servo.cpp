//
// Created by Jasper Cusiel on 20/07/2026.
//

#include <smart-servo.h>
#include <HerkulexServo.h>
#include "display.h"

// The module controls the herkulex smart servos.

#define SERIAL_BUS Serial2
#define SERIAL_BAUD 115200 // max is 0.67 MBPS
#define SERVO_ID_A 02
#define SERVO_ID_B 01


//need to set these values
#define FRONT_SERVO_UP_POS   600
#define FRONT_SERVO_DOWN_POS 800//400
#define BACK_SERVO_UP_POS    700
#define BACK_SERVO_DOWN_POS  600
#define SERVO_MOVE_PLAYTIME  50
#define SERVO_REBOOT_RECOVERY_DELAY_MS 500UL

HerkulexServoBus herkulex_bus(SERIAL_BUS);
HerkulexServo servo_a(herkulex_bus, SERVO_ID_A);
HerkulexServo servo_b(herkulex_bus, SERVO_ID_B);

// All servos daisy chained
HerkulexServo* servos[] = {&servo_a, &servo_b};
uint8_t servo_id[] = {SERVO_ID_A, SERVO_ID_B};

HerkulexStatusError servo_error;
HerkulexStatusDetail detail;

// Up/down tracking
bool servo_down[] = {false, false}; // back, front
bool servo_recovering[] = {false, false};
uint32_t servo_reboot_time_ms[] = {0, 0};

static bool read_servo_status(size_t servo_index, HerkulexStatusError& status_error,
                              HerkulexStatusDetail& status_detail)
{
    HerkulexPacket resp;
    if (!herkulex_bus.sendPacketAndReadResponse(resp, servo_id[servo_index], HerkulexCommand::Stat))
    {
        return false;
    }

    status_error = static_cast<HerkulexStatusError>(resp.data[0]);
    status_detail = static_cast<HerkulexStatusDetail>(resp.data[1]);
    // Serial.printf("status: %d\n", status_error);
    // Serial.printf("detail: %d\n", status_detail);

    return true;
}

static void restore_servo_target(size_t servo_index)
{
    if (servo_index == 0)
    {
        servos[servo_index]->setPosition(servo_down[servo_index] ? BACK_SERVO_DOWN_POS : BACK_SERVO_UP_POS,
                                         SERVO_MOVE_PLAYTIME);
    }
    else
    {
        servos[servo_index]->setPosition(servo_down[servo_index] ? FRONT_SERVO_DOWN_POS : FRONT_SERVO_UP_POS,
                                         SERVO_MOVE_PLAYTIME);
    }
}

bool smart_servo_init()
{
    SERIAL_BUS.begin(SERIAL_BAUD);
    uint8_t num_errors = 0;

    for (size_t i = 0; i < 2; i++)
    {
        if (!read_servo_status(i, servo_error, detail))
        {
            char buf[17];
            snprintf(buf, sizeof(buf), "SV%d NO RESP", i);
            display_log(buf);
            num_errors++;
            continue;
        }

        char buf[17];
        snprintf(buf, sizeof(buf), "SV%d E:%02X D:%02X", i,
                 (uint8_t)servo_error, (uint8_t)detail);
        display_log(buf);

        servos[i]->setLedColor(HerkulexLed::Green);
        servos[i]->setTorqueOn();
        servos[i]->enablePositionControlMode();

        if (servo_error != HerkulexStatusError::None)
        {
            num_errors++;
        }
    }
    set_front_servo_down();
    set_back_servo_down();

    return num_errors == 0;
}

void set_front_servo_up()
{
    servo_b.setPosition(FRONT_SERVO_UP_POS, SERVO_MOVE_PLAYTIME);
    servo_down[1] = false;
}

void set_front_servo_down()
{
    servo_b.setPosition(FRONT_SERVO_DOWN_POS, SERVO_MOVE_PLAYTIME);
    servo_down[1] = true;
}

void set_back_servo_up()
{
    servo_a.setPosition(BACK_SERVO_UP_POS, SERVO_MOVE_PLAYTIME);
    servo_down[0] = false;
}

void set_back_servo_down()
{
    servo_a.setPosition(BACK_SERVO_DOWN_POS, SERVO_MOVE_PLAYTIME);
    servo_down[0] = true;
}

bool is_front_servo_in_position()
{
    HerkulexStatusError status_error;
    HerkulexStatusDetail status_detail;
    servo_b.getStatus(status_error, status_detail);
    return (status_detail & HerkulexStatusDetail::InPosition) != HerkulexStatusDetail::None;
}

bool is_back_servo_in_position()
{
    HerkulexStatusError status_error;
    HerkulexStatusDetail status_detail;
    servo_a.getStatus(status_error, status_detail);
    return (status_detail & HerkulexStatusDetail::InPosition) != HerkulexStatusDetail::None;
}

void smart_servo_monitor_task()
{
    HerkulexStatusError status_error;
    HerkulexStatusDetail status_detail;

    for (size_t i = 0; i < 2; i++)
    {
        if (servo_recovering[i])
        {
            if (millis() - servo_reboot_time_ms[i] < SERVO_REBOOT_RECOVERY_DELAY_MS)
            {
                continue;
            }

            if (!read_servo_status(i, status_error, status_detail))
            {
                continue;
            }

            if (status_error != HerkulexStatusError::None)
            {
                servos[i]->reboot();
                servo_reboot_time_ms[i] = millis();
                continue;
            }

            servos[i]->setLedColor(HerkulexLed::Green);
            servos[i]->setTorqueOn();
            servos[i]->enablePositionControlMode();
            restore_servo_target(i);
            servo_recovering[i] = false;
            continue;
        }

        if (!read_servo_status(i, status_error, status_detail))
        {
            continue;
        }

        if (status_error != HerkulexStatusError::None)
        {
            servos[i]->reboot();
            servo_recovering[i] = true;
            servo_reboot_time_ms[i] = millis();
        }
    }
}
