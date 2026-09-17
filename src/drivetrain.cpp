//
// Created by Jasper Cusiel on 06/09/2026.
//

#include "drivetrain.h"

#include <Arduino.h>
#include <Servo.h>
#include <DCMotorServo.h>
#include <DCMotorTacho.h>
#include <Encoder.h>


namespace
{
    // Motor drive config
    constexpr uint8_t kLeftPwmPin = 28;
    constexpr uint8_t kLeftENCA = 30;
    constexpr uint8_t kLeftENCB = 31;

    constexpr uint8_t kRightPwmPin = 1;
    constexpr uint8_t kRightENCA = 2;
    constexpr uint8_t kRightENCB = 3;


    constexpr int kFullForwardUs = 1950;
    constexpr int kFullReverseUs = 1050;
    constexpr int kStopUs = 1500;
    constexpr int kPWM_MAX = 255;

    // TODO: Calibrate these for the actual drivetrain hardware.
    constexpr float kWheelDiameterM = 0.065f;
    constexpr float kWheelCircumferenceM = PI * kWheelDiameterM;
    constexpr double kEncoderCpr = 3540;
    constexpr unsigned long kSpeedControlIntervalMs = 50;

    constexpr uint8_t kPwmSkip = 120;
    constexpr uint8_t kPositionAccuracyCounts = 50;

    constexpr double kPositionKp = 0.1;
    constexpr double kPositionKi = 0.05;
    constexpr double kPositionKd = 0.0;

    constexpr double kSpeedKp = 10.0;
    constexpr double kSpeedKi = 0.0;
    constexpr double kSpeedKd = 0.0;

    double right_outer_kp = kPositionKp;
    double right_outer_ki = kPositionKi;
    double right_outer_kd = kPositionKd;
    double right_inner_kp = kSpeedKp;
    double right_inner_ki = kSpeedKi;
    double right_inner_kd = kSpeedKd;
    bool right_position_tuning_mode = false;

    template <Servo& Motor, Encoder& EncoderInput, bool MotorInverted>
    struct MotorCallbacks
    {
        static void write(int16_t speed)
        {
            const int16_t driver_speed = MotorInverted ? -speed : speed;
            const int pulse = map(driver_speed, -kPWM_MAX, kPWM_MAX, kFullReverseUs, kFullForwardUs);
            Motor.writeMicroseconds(pulse);
        }

        static void brake() { Motor.writeMicroseconds(kStopUs); }
        static long readEncoder() { return EncoderInput.read(); }
        static void writeEncoder(long position) { EncoderInput.write(position); }
    };

    // Motor driver uses PPM control (essentially the same as a servo)
    // Motor drives are PPM controlled (thus the Servo library generates pulses).
    Servo left_driver;
    Servo right_driver;
    Encoder left_enc(kLeftENCA, kLeftENCB);
    Encoder right_enc(kRightENCA, kRightENCB);

    using Motor1Callbacks = MotorCallbacks<left_driver, left_enc, true>;
    using Motor2Callbacks = MotorCallbacks<right_driver, right_enc, false>;

    DCMotorServo left_motor(
        Motor1Callbacks::write,
        Motor1Callbacks::brake,
        Motor1Callbacks::readEncoder,
        Motor1Callbacks::writeEncoder);

    DCMotorServo right_motor(
        Motor2Callbacks::write,
        Motor2Callbacks::brake,
        Motor2Callbacks::readEncoder,
        Motor2Callbacks::writeEncoder);

    DCMotorTacho left_tacho(&left_motor, kEncoderCpr, kSpeedControlIntervalMs);
    DCMotorTacho right_tacho(&right_motor, kEncoderCpr, kSpeedControlIntervalMs);

    double wheel_speed_to_rpm(float wheel_speed_mps)
    {
        return static_cast<double>(wheel_speed_mps / kWheelCircumferenceM * 60.0f);
    }

    void configure_motor(DCMotorServo& motor, DCMotorTacho& tacho)
    {
        motor.setPWMSkip(kPwmSkip);
        motor.setAccuracy(kPositionAccuracyCounts);
        motor.setMaxPWM(kPWM_MAX);
        motor.setPIDTunings(kPositionKp, kPositionKi, kPositionKd);
        tacho.setPIDTunings(kSpeedKp, kSpeedKi, kSpeedKd);
    }

    void print_right_tuning_help()
    {
        Serial.println("Right drivetrain speed PID tuning");
        Serial.println(
            "Commands: SPEED=<rpm>, MOVE=<counts>, MOVETO=<counts>, SPEEDKP=<kp>, SPEEDKI=<ki>, SPEEDKD=<kd>, KP=<kp>, KI=<ki>, KD=<kd>, STOP");
        Serial.println("Plotter: outer_setpoint,outer_input,outer_output,MeasuredSpeedRPM,DesiredSpeedRPM");
    }
} // namespace

void drivetrain_init()
{
    // Initialize motor drives
    left_driver.attach(kLeftPwmPin, kFullReverseUs, kFullForwardUs);
    right_driver.attach(kRightPwmPin, kFullReverseUs, kFullForwardUs);

    configure_motor(left_motor, left_tacho);
    configure_motor(right_motor, right_tacho);

    // Make sure we don't command any motor speed on startup.
    set_wheel_speed_targets(0.0f, 0.0f);
}

void drivetrain_update()
{
    left_tacho.run();
    right_tacho.run();
}

void set_wheel_speed_targets(float left_mps, float right_mps)
{
    left_tacho.setSpeedRPM(wheel_speed_to_rpm(left_mps));
    right_tacho.setSpeedRPM(wheel_speed_to_rpm(right_mps));
}

void set_motor_speeds(float left_mps, float right_mps)
{
    set_wheel_speed_targets(left_mps, right_mps);
}

void PID_tune()
{
    static bool printed_help = false;
    static unsigned long lastPrintTime = millis();

    if (!printed_help)
    {
        print_right_tuning_help();
        printed_help = true;
    }

    // if (right_position_tuning_mode)
    // {
    //     right_motor.run();
    // }
    // else
    // {
    //     right_tacho.run();
    // }
    right_tacho.run();

    // Print status every 50ms.
    if (millis() - lastPrintTime > 300)
    {
        lastPrintTime = millis();
        double measuredSpeed = right_tacho.getMeasuredSpeedRPM();
        double desiredSpeed = right_tacho.getDesiredSpeedRPM();
        Serial.print(right_tacho.getServo()->getSerialPlotter());
        Serial.print(", MeasuredSpeedRPM:");
        Serial.print(measuredSpeed, 2);
        Serial.print(", DesiredSpeedRPM:");
        Serial.println(desiredSpeed, 2);
    }

    // Process serial commands for tuning.
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("SPEED="))
        {
            double rpm = command.substring(6).toFloat();
            right_position_tuning_mode = false;
            right_tacho.setSpeedRPM(rpm);
            Serial.print("Desired Speed set to: ");
            Serial.println(rpm);
        }
        else if (command.startsWith("SPEEDKP="))
        {
            double kp = command.substring(8).toFloat();
            right_inner_kp = kp;
            right_tacho.setPIDTunings(right_inner_kp, right_inner_ki, right_inner_kd);
            Serial.print("Inner PID Kp set to: ");
            Serial.println(kp);
        }
        else if (command.startsWith("SPEEDKI="))
        {
            double ki = command.substring(8).toFloat();
            right_inner_ki = ki;
            right_tacho.setPIDTunings(right_inner_kp, right_inner_ki, right_inner_kd);
            Serial.print("Inner PID Ki set to: ");
            Serial.println(ki);
        }
        else if (command.startsWith("SPEEDKD="))
        {
            double kd = command.substring(8).toFloat();
            right_inner_kd = kd;
            right_tacho.setPIDTunings(right_inner_kp, right_inner_ki, right_inner_kd);
            Serial.print("Inner PID Kd set to: ");
            Serial.println(kd);
        }
        // Outer loop tuning commands.
        else if (command.startsWith("KP="))
        {
            right_outer_kp = command.substring(3).toFloat();
            right_motor.setPIDTunings(right_outer_kp, right_outer_ki, right_outer_kd);
            Serial.print("Outer PID Kp set to: ");
            Serial.println(right_outer_kp);
        }
        else if (command.startsWith("KI="))
        {
            right_outer_ki = command.substring(3).toFloat();
            right_motor.setPIDTunings(right_outer_kp, right_outer_ki, right_outer_kd);
            Serial.print("Outer PID Ki set to: ");
            Serial.println(right_outer_ki);
        }
        else if (command.startsWith("KD="))
        {
            right_outer_kd = command.substring(3).toFloat();
            right_motor.setPIDTunings(right_outer_kp, right_outer_ki, right_outer_kd);
            Serial.print("Outer PID Kd set to: ");
            Serial.println(right_outer_kd);
        }
        else if (command.startsWith("STOP"))
        {
            right_tacho.stop();
            right_position_tuning_mode = false;
            Serial.println("Motor stopped.");
        }
        // else if (command.startsWith("MOVE="))
        // {
        //     long counts = command.substring(5).toInt();
        //     right_tacho.stop();
        //     right_position_tuning_mode = true;
        //     right_motor.move(counts);
        //     Serial.print("Move by counts: ");
        //     Serial.println(counts);
        // }
        // else if (command.startsWith("MOVETO="))
        // {
        //     long position = command.substring(7).toInt();
        //     right_tacho.stop();
        //     right_position_tuning_mode = true;
        //     right_motor.moveTo(position);
        //     Serial.print("Move to count: ");
        //     Serial.println(position);
        // }
        else
        {
            print_right_tuning_help();
        }
    }
}
