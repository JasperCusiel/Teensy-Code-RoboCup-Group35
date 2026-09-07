//
// Created by Jasper Cusiel on 21/07/2026.
//
#include  "lift-motor.h"
#include <Servo.h>
#include <Encoder.h>
#include "limit-switch.h"

// Lifter motor config
// Motor 1
#define PWM1_PIN 28
#define ENC1_A   32
#define ENC1_B   33

// Motor 2
#define PWM2_PIN 29
#define ENC2_A   30
#define ENC2_B   31

// Homing switches
#define END_STOP_A_PIN 0
#define END_STOP_B_PIN 1

// Homing and travel parameters
#define MAX_TRAVEL_ENC_COUNT 40000
#define FULL_FORWARD 1950
#define FULL_REVERSE 1050
#define STOP 1500
#define PWM_MAX 255
#define HOMING_SPEED 200
#define LIMIT_SWITCH_BACK_OFF 2000
#define PWM_SKIP 71
#define ACCURACY 50

// PID tuning values
#define SERVO_KP 0.1
#define SERVO_KI 0.1
#define SERVO_KD 0.05

namespace
{
    // DCMotorServo takes plain function pointers, which cannot carry a Servo or
    // Encoder reference. Template generates the callbacks for each pair.
    template <Servo& Motor, Encoder& EncoderInput>
    struct MotorCallbacks
    {
        static void write(int16_t speed)
        {
            const int pulse = map(speed, -PWM_MAX, PWM_MAX, FULL_REVERSE, FULL_FORWARD);
            Motor.writeMicroseconds(pulse);
        }

        static void brake() { Motor.writeMicroseconds(STOP); }
        static long readEncoder() { return EncoderInput.read(); }
        static void writeEncoder(long position) { EncoderInput.write(position); }
    };

    template <uint8_t Pin>
    bool readEndstop()
    {
        return readLimitSwitch(Pin);
    }

    // Motor drives are PPM controlled (thus the Servo library generates pulses).
    Servo motor1;
    Servo motor2;
    Encoder enc1(ENC1_A, ENC1_B);
    Encoder enc2(ENC2_A, ENC2_B);

    using Motor1Callbacks = MotorCallbacks<motor1, enc1>;
    using Motor2Callbacks = MotorCallbacks<motor2, enc2>;

    DCMotorServo servo1(
        Motor1Callbacks::write,
        Motor1Callbacks::brake,
        Motor1Callbacks::readEncoder,
        Motor1Callbacks::writeEncoder);

    DCMotorServo servo2(
        Motor2Callbacks::write,
        Motor2Callbacks::brake,
        Motor2Callbacks::readEncoder,
        Motor2Callbacks::writeEncoder);
}

static bool home_servo(DCMotorServo* servo)
{
    // Function homes the lifter motor by first backing off, fast homing, backing off then slow homing (much like a 3D printer or cnc).

    // Move away from limit switch first
    servo->setCurrentPosition(0);
    servo->moveTo(-LIMIT_SWITCH_BACK_OFF);

    while (!servo->finished())
    {
        servo->run();
    }

    // First fast homing
    servo->startHoming(1, HOMING_SPEED, MAX_TRAVEL_ENC_COUNT);

    while (servo->isHoming())
    {
        servo->run();
    }
    // Back off from switch
    servo->moveTo(-LIMIT_SWITCH_BACK_OFF);

    while (!servo->finished())
    {
        servo->run();
    }

    // Slow homing
    servo->startHoming(1, 100, MAX_TRAVEL_ENC_COUNT);

    while (servo->isHoming())
    {
        servo->run();
    }

    if (servo->isHomed())
    {
        servo->setTravelLimits(0, MAX_TRAVEL_ENC_COUNT);
        return true;
    }

    return false;
}


bool lifter_motor_init()
{
    // Function initializes the motor drives and servo objects
    motor1.attach(PWM1_PIN);
    motor2.attach(PWM2_PIN);

    // Setup servo config
    servo1.setPWMSkip(PWM_SKIP); // minimum PWM to overcome stiction
    servo1.setAccuracy(10); // acceptable position error in counts
    servo1.setMaxPWM(PWM_MAX);
    servo1.setPIDTunings(SERVO_KP, SERVO_KI, SERVO_KD);
    servo1.attachEndstops(nullptr, readEndstop<END_STOP_A_PIN>);

    servo2.setPWMSkip(PWM_SKIP); // minimum PWM to overcome stiction
    servo2.setAccuracy(10); // acceptable position error in counts
    servo2.setMaxPWM(PWM_MAX);
    servo2.setPIDTunings(SERVO_KP, SERVO_KI, SERVO_KD);
    servo2.attachEndstops(nullptr, readEndstop<END_STOP_B_PIN>);


    if (home_servo(&servo1) && home_servo(&servo2))
    {
        return true;
    }

    return false;
}
