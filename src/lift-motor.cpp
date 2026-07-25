//
// Created by Jasper Cusiel on 21/07/2026.
//
#include  <lift-motor.h>

#include <limit-switch.h>

// Motor 1
#define PWM1_PIN 28
#define ENC1_A   32
#define ENC1_B   33

// Motor 2
#define PWM2_PIN 29
#define ENC2_A   30
#define ENC2_B   31


#define END_STOP_A_PIN 0
#define END_STOP_B_PIN 1

#define MAX_TRAVEL_ENC_COUNT 40000
#define FULL_FORWARD 1950
#define FULL_REVERSE 1050
#define STOP 1500
#define PWM_MAX 255
#define HOMING_SPEED 200
#define LIMIT_SWITCH_BACK_OFF 2000
#define PWM_SKIP 71
#define ACCURACY 50



// PWM skip tuning
// ----- Tuning Constants -----
const unsigned long INIT_PWM_RAMP_INTERVAL = 400; // Initial interval (ms) between PWM increments
const int MOVE_STEPS_SENSITIVITY = 5;             // Minimum encoder steps to consider as movement

// ----- Global Variables for Tuning -----
unsigned long pwmRampInterval; // Current PWM ramp interval
int currentPWM;                // Current PWM value being applied
int pwmStart;                  // Starting PWM for the current iteration
int pwmEstimate;               // Latest PWM estimate from encoder detection
int pwmEstimate0;              // PWM value recorded when movement is first detected

// Timers for independent branches
unsigned long lastPWMUpdateTime = 0;
unsigned long lastEncoderCheckTime = 0;

// Accuracy estimate tuning
// ----- Tuning Constants -----
const int INIT_ACCURACY = 420;                    // Initial accuracy threshold (adjust as needed)
const int targetSteps = 5000;                    // Target steps for the motor to reach
const unsigned long STABILIZATION_PERIOD = 3000; // Time (ms) for stabilization control once target is passed
const int TRIALS_PER_ACCURACY = 5;               // Number of consecutive successful stabilization trials required per accuracy level
const unsigned long SYSTEM_DELAY = 2;            // Control update interval in ms to reflect the expected system delay

// ----- Global Variables for Tuning -----
int currentAccuracy; // Current accuracy threshold
long pos = 0;        // Variable to store encoder position


// PID tuning
#define SERVO_KP 0.1
#define SERVO_KI 0.1
#define SERVO_KD 0.05

// PID parameters for motor servo object
float KP = SERVO_KP;
float KI = SERVO_KI;
float KD = SERVO_KD;

Encoder  enc1(ENC1_A, ENC1_B);
Servo motor1;

Encoder  enc2(ENC2_A, ENC2_B);
Servo motor2;


void motor1Write(int16_t s) {
  int pulse = map(s, -PWM_MAX, PWM_MAX, FULL_REVERSE, FULL_FORWARD);
  motor1.writeMicroseconds(pulse);
}
void motor1Brake()           { motor1.writeMicroseconds(STOP);}
long enc1Read()              { return enc1.read(); }
void enc1Write(long position)       { enc1.write(position); }
bool getLimitSwitch1() {
  return readLimitSwitch(END_STOP_A_PIN);
}

void motor2Write(int16_t s) {
  int pulse = map(s, -PWM_MAX, PWM_MAX, FULL_REVERSE, FULL_FORWARD);
  motor2.writeMicroseconds(pulse);
}
void motor2Brake()           { motor2.writeMicroseconds(STOP);}
long enc2Read()              { return enc2.read(); }
void enc2Write(long position)       { enc2.write(position); }
bool getLimitSwitch2() {
  return readLimitSwitch(END_STOP_B_PIN);
}



DCMotorServo servo1(motor1Write, motor1Brake, enc1Read, enc1Write);
DCMotorServo servo2(motor2Write, motor2Brake, enc2Read, enc2Write);

bool home_servo(DCMotorServo *servo) {
  Serial.println("Back off");
  // Move away from limit switch first
  servo->setCurrentPosition(0);
  servo->moveTo(-LIMIT_SWITCH_BACK_OFF);

  while (!servo->finished()) {
    Serial.println(enc1.read());
    servo->run();
  }

  // First fast homing
  if (servo->startHoming(1, HOMING_SPEED, MAX_TRAVEL_ENC_COUNT)) {
    Serial.println("Fast homing...");
  }

  while (servo->isHoming()) {
    servo->run();
  }

  Serial.println("Back off");
  // Back off from switch
  servo->moveTo(-LIMIT_SWITCH_BACK_OFF);

  while (!servo->finished()) {
    servo->run();
  }

  Serial.println("Slow homing");
  if (servo->startHoming(1, 100, MAX_TRAVEL_ENC_COUNT)) {
    Serial.println("Lifter Motor A Homing slow...");
  }

  while (servo->isHoming()) {
    servo->run();
  }

  if (servo->isHomed()) {
    servo->setTravelLimits(0, MAX_TRAVEL_ENC_COUNT);
    Serial.println("Lifter motor A homed.");
    return true;
  }

  return false;

}


bool lifter_motor_init() {

  motor1.attach(PWM1_PIN);

  motor2.attach(PWM2_PIN);

  servo1.setPWMSkip(PWM_SKIP);       // minimum PWM to overcome stiction
  servo1.setAccuracy(10);      // acceptable position error in counts
  servo1.setMaxPWM(PWM_MAX);
  servo1.setPIDTunings(0.15, 0.1, 0.001);
  servo1.attachEndstops(nullptr, &getLimitSwitch1);

  servo2.setPWMSkip(PWM_SKIP);       // minimum PWM to overcome stiction
  servo2.setAccuracy(10);      // acceptable position error in counts
  servo2.setMaxPWM(PWM_MAX);
  servo2.setPIDTunings(0.15, 0.1, 0.001);
  servo2.attachEndstops(nullptr, &getLimitSwitch2);

  //pwm_skip_tuning(&motor2, PWM2_PIN, &enc2, motor2Brake, motor2Write);
  //pwm_skip_tuning(&motor1, PWM1_PIN, &enc1, motor1Brake, motor1Write);

  if (home_servo(&servo1)) {
    return true;
  }

  return false;
}


// PWM skip tuning
void serialPrintPWMSkipResults()
{
  Serial.print("PWM_Start: ");
  Serial.print(pwmStart);
  Serial.print(", Current_PWM: ");
  Serial.print(currentPWM);
  Serial.print(", PWM_Estimate: ");
  Serial.print(pwmEstimate);
  Serial.print(", Ramp_Interval: ");
  Serial.print(pwmRampInterval);
  Serial.print(", Encoder: ");
  Serial.println(enc1.read());
}

void pwm_skip_tuning(Servo *motor, uint8_t pwm_pin, Encoder *encoder, void (*motorBreak)(), void (*motorWrite)(int16_t s)) {
  motor->attach(pwm_pin);
  encoder->write(0); // Zero the encoder

  // Initialize tuning variables
  pwmRampInterval = INIT_PWM_RAMP_INTERVAL;
  pwmStart = currentPWM = 0;
  pwmEstimate = PWM_MAX;

  lastPWMUpdateTime = millis();
  lastEncoderCheckTime = millis();

  Serial.println("PWM Skip Tuning Procedure Start");
  Serial.println("PWM_Start, PWM_Estimate, Ramp_Interval");

  // Apply a brake and pause to allow the motor to settle.
  motorBreak();
  delay(1000);

  while (true) {
    static bool tuningComplete = false;
    unsigned long currentTime = millis();

    // Terminate tuning once the estimate converges
    if (tuningComplete)
    {
      Serial.print("Tuning complete. Final pwmStart: ");
      Serial.print(pwmStart);
      Serial.print(", Final pwmEstimate: ");
      Serial.println(pwmEstimate);
      motorBreak();
      while (1)
        ; // Halt execution
    }

    long encoderMovement = abs(encoder->read());
    if (encoderMovement >= MOVE_STEPS_SENSITIVITY)
    {
      // Significant movement detected.
      pwmEstimate0 = currentPWM;
      pwmEstimate = pwmEstimate0;

      // Update pwmStart to halfway between previous pwmStart and new estimate.
      pwmStart = pwmEstimate0 - ((pwmEstimate0 - pwmStart) / 2);

      // Double the ramp interval for the next iteration.
      pwmRampInterval *= 2;

      serialPrintPWMSkipResults();

      // Apply a brake and pause to allow the motor to settle.
      motorBreak();
      delay(1000);

      // Reset encoder for the next iteration.
      encoder->write(0);
      // Reset current PWM to new start value.
      currentPWM = pwmStart;

      // Check for convergence.
      if (pwmEstimate == pwmStart)
      {
        tuningComplete = true;
      }
    }

    // ---- PWM Ramp Update Branch (every pwmRampInterval ms) ----
    else if (currentTime - lastPWMUpdateTime >= pwmRampInterval)
    {
      lastPWMUpdateTime = currentTime;
      motorWrite((int16_t)currentPWM);

      // Only ramp up PWM if encoder movement is below sensitivity.
      if (abs(encoder->read()) < MOVE_STEPS_SENSITIVITY)
      {
        currentPWM++;
        if (currentPWM > PWM_MAX)
        {
          currentPWM = PWM_MAX;
        }
        serialPrintPWMSkipResults();
      }
    }
  }
}

// Accuracy tuning

void serialPrintAccuracyResults()
{
  Serial.print("Current Accuracy: ");
  Serial.print(currentAccuracy);
  Serial.print(", Target Steps: ");
  Serial.print(targetSteps);
  Serial.print(", Encoder Position: ");
  Serial.println(pos);
}

void accuracy_estimate_tuning() {
  motor1.attach(PWM1_PIN);
  enc1.write(0); // Zero the encoder
  // servo.setPWMSkip(PWM_START);

  // Initialize accuracy tuning parameters
  currentAccuracy = INIT_ACCURACY;

  Serial.println("Accuracy Estimate Tuning Procedure Start");
  while (true) {
    // Stabilization trials with bang-bang control.
    Serial.println("Starting stabilization trials...");
    int successCount = 0;
    int errorCount = 0;

    for (int i = 0; i < TRIALS_PER_ACCURACY; i++)
    {
        // Reset encoder for this trial.
        enc1.write(0);

        // Run stabilization control for a fixed period.
        unsigned long stabilizeStart = millis();
        unsigned long lastSerialPrintTime = millis(); // For non-blocking Serial printing.
        unsigned long lastControlTime = millis();     // For SYSTEM_DELAY control updates.
        while (millis() - stabilizeStart < STABILIZATION_PERIOD)
        {
            pos = enc1.read();

            // Only update control every SYSTEM_DELAY ms.
            if (millis() - lastControlTime >= SYSTEM_DELAY)
            {
                lastControlTime = millis();
                if (pos < targetSteps - currentAccuracy)
                {
                    motor1Write(PWM_MAX);
                }
                else if (pos > targetSteps + currentAccuracy)
                {
                    motor1Write(-PWM_MAX);
                }
                else
                {
                    motor1Brake();
                }
            }

            // Only print results every 100 ms.
            if (millis() - lastSerialPrintTime >= 100)
            {
                serialPrintAccuracyResults();
                lastSerialPrintTime = millis();
            }
        }

        motor1Write(0); // Release the motor and allow it to stop.

        // Report trial results.
        long finalError = abs(enc1.read() - targetSteps);
        if (finalError <= currentAccuracy)
        {
            successCount++;
            Serial.print("Trial successful. ");
            Serial.println(successCount);
        }
        else
        {
            errorCount++;
            Serial.print("Trial failed. ");
            Serial.println(finalError);
        }
        Serial.println("Successes: " + String(successCount) + ", Failures: " + String(errorCount));
    }

    // If all trials failed at this accuracy, end the procedure.
    if (errorCount >= successCount)

    {
        Serial.println("Stabilization failed at current accuracy.");
        Serial.print("Final achieved accuracy: ");
        Serial.println(currentAccuracy);
        motor1Write(0); // Release the motor and allow it to stop.
        while (1)
            ; // Halt execution.
    }

    // Refine accuracy threshold.
    int previousAccuracy = currentAccuracy;
    currentAccuracy = currentAccuracy - 1;
    if (currentAccuracy < 1)
    {
        currentAccuracy = 1;
    }

    Serial.print("Refining accuracy: previous = ");
    Serial.print(previousAccuracy);
    Serial.print(", new = ");
    Serial.println(currentAccuracy);

    enc1.write(0);
    delay(2000);

    if (previousAccuracy - currentAccuracy <= 0)
    {
        Serial.println("Final achieved accuracy:");
        Serial.println(previousAccuracy);
        motor1Brake();
        while (1)
            ; // Halt execution.
    }
  }
}

void pid_tuning() {
  motor1.attach(PWM1_PIN);
  enc1.write(0); // Zero the encoder

  // Configure the PID controller
  servo1.myPID->SetTunings(KP, KI, KD);
  servo1.setPWMSkip(PWM_SKIP);
  servo1.setAccuracy(ACCURACY);
  servo1.setMaxPWM(PWM_MAX);

  Serial.println("Setpoint,Input,Output");

  while (true) {
    static unsigned long lastPrintTime = millis();

    // Run the PID loop
    servo1.run();

    // Print PID values for tuning via Serial Plotter every 20ms
    if (millis() - lastPrintTime > 20)
    {
        lastPrintTime = millis();
        Serial.println(servo1.getSerialPlotter());
    }

    // Allow PID tuning commands via Serial
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("KP="))
        {
            KP = command.substring(3).toFloat();
            servo1.myPID->SetTunings(KP, KI, KD);
            Serial.print("KP set to: ");
            Serial.println(KP);
        }
        else if (command.startsWith("KI="))
        {
            KI = command.substring(3).toFloat();
            servo1.myPID->SetTunings(KP, KI, KD);
            Serial.print("KI set to: ");
            Serial.println(KI);
        }
        else if (command.startsWith("KD="))
        {
            KD = command.substring(3).toFloat();
            servo1.myPID->SetTunings(KP, KI, KD);
            Serial.print("KD set to: ");
            Serial.println(KD);
        }
        else if (command.startsWith("MOVE="))
        {
            long newPosition = command.substring(5).toInt();
            if (newPosition != 0)
            {
                servo1.move(newPosition);
                Serial.print("Moving to: ");
                Serial.println(newPosition);
            }
            else
            {
                Serial.println("Invalid MOVE command");
            }
        }
        else if (command.startsWith("MAXPWM="))
        {
            int maxPWM = command.substring(7).toInt();
            if (maxPWM >= 0 && maxPWM <= 255)
            {
                servo1.setMaxPWM(maxPWM);
                Serial.print("Setting MAXPWM to: ");
                Serial.println(maxPWM);
            }
            else
            {
                Serial.println("Invalid MAXPWM command");
            }
        }
        else
        {
            Serial.println("Invalid command");
        }
    }
  }
}