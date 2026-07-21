//
// Created by Jasper Cusiel on 21/07/2026.
//
#include  <lift-motor.h>
#include <DCMotorServo.h>
#include <Encoder.h>
#include <Servo.h>
#include <limit-switch.h>


#define PWM_PIN 28
#define ENC1_A   32
#define ENC1_B   33
#define END_STOP_PIN 0
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

Encoder  enc(ENC1_A, ENC1_B);
Servo motor1;



void motorWrite(int16_t s) {
  int pulse = map(s, -PWM_MAX, PWM_MAX, FULL_REVERSE, FULL_FORWARD);
  motor1.writeMicroseconds(pulse);
}
void motorBrake()           { motor1.writeMicroseconds(STOP);}
long encRead()              { return enc.read(); }
void encWrite(long position)       { enc.write(position); }
bool getLimitSwitch() {
  return readLimitSwitch(END_STOP_PIN);
}



DCMotorServo servo(motorWrite, motorBrake, encRead, encWrite);

bool lifter_motor_init() {
  motor1.attach(PWM_PIN);
  pinMode(END_STOP_PIN, INPUT);

  //servo.setPWMSkip(25);       // minimum PWM to overcome stiction
  servo.setAccuracy(10);      // acceptable position error in counts
  servo.setMaxPWM(PWM_MAX);
  servo.setPIDTunings(0.15, 0.1, 0.001);
  servo.attachEndstops(nullptr, &getLimitSwitch);

  Serial.println("Back off");
  // Move away from limit switch first
  servo.setCurrentPosition(0);
  servo.moveTo(-LIMIT_SWITCH_BACK_OFF);

  while (!servo.finished()) {
    servo.run();
  }

  // First fast homing
  if (servo.startHoming(1, HOMING_SPEED, MAX_TRAVEL_ENC_COUNT)) {
    Serial.println("Fast homing...");
  }

  while (servo.isHoming()) {
    servo.run();
  }

  Serial.println("Back off");
  // Back off from switch
  servo.moveTo(-LIMIT_SWITCH_BACK_OFF);

  while (!servo.finished()) {
    servo.run();
  }

  Serial.println("Slow homing");
  if (servo.startHoming(1, 100, MAX_TRAVEL_ENC_COUNT)) {
    Serial.println("Lifter Motor Homing slow...");
  }

  while (servo.isHoming()) {
    servo.run();
  }

  if (servo.isHomed()) {
    servo.setTravelLimits(0, MAX_TRAVEL_ENC_COUNT);
    Serial.println("Lifter motor homed.");
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
  Serial.println(enc.read());
}

void pwm_skip_tuning() {
  motor1.attach(PWM_PIN);
  enc.write(0); // Zero the encoder

  // Initialize tuning variables
  pwmRampInterval = INIT_PWM_RAMP_INTERVAL;
  pwmStart = currentPWM = 0;
  pwmEstimate = PWM_MAX;

  lastPWMUpdateTime = millis();
  lastEncoderCheckTime = millis();

  Serial.println("PWM Skip Tuning Procedure Start");
  Serial.println("PWM_Start, PWM_Estimate, Ramp_Interval");

  // Apply a brake and pause to allow the motor to settle.
  motorBrake();
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
      motorBrake();
      while (1)
        ; // Halt execution
    }

    long encoderMovement = abs(enc.read());
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
      motorBrake();
      delay(1000);

      // Reset encoder for the next iteration.
      enc.write(0);
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
      motorWrite(currentPWM);

      // Only ramp up PWM if encoder movement is below sensitivity.
      if (abs(enc.read()) < MOVE_STEPS_SENSITIVITY)
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
  motor1.attach(PWM_PIN);
  enc.write(0); // Zero the encoder
  // servo.setPWMSkip(PWM_START);

  // Initialize accuracy tuning parameters
  currentAccuracy = INIT_ACCURACY;

  Serial.println("Accuracy Estimate Tuning Procedure Start");
  while (true) {
    // Stabilization trials with bang-bang control.
    Serial.println("Starting stabilization trials...");
    int successCount = 0;
    int errorCount = 0;
    bool anyTrialSuccess = false; // Track if any trial succeeded

    for (int i = 0; i < TRIALS_PER_ACCURACY; i++)
    {
        // Reset encoder for this trial.
        enc.write(0);

        // Run stabilization control for a fixed period.
        unsigned long stabilizeStart = millis();
        unsigned long lastSerialPrintTime = millis(); // For non-blocking Serial printing.
        unsigned long lastControlTime = millis();     // For SYSTEM_DELAY control updates.
        while (millis() - stabilizeStart < STABILIZATION_PERIOD)
        {
            pos = enc.read();

            // Only update control every SYSTEM_DELAY ms.
            if (millis() - lastControlTime >= SYSTEM_DELAY)
            {
                lastControlTime = millis();
                if (pos < targetSteps - currentAccuracy)
                {
                    motorWrite(PWM_MAX);
                }
                else if (pos > targetSteps + currentAccuracy)
                {
                    motorWrite(-PWM_MAX);
                }
                else
                {
                    motorBrake();
                }
            }

            // Only print results every 100 ms.
            if (millis() - lastSerialPrintTime >= 100)
            {
                serialPrintAccuracyResults();
                lastSerialPrintTime = millis();
            }
        }

        motorWrite(0); // Release the motor and allow it to stop.

        // Report trial results.
        long finalError = abs(enc.read() - targetSteps);
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
        motorWrite(0); // Release the motor and allow it to stop.
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

    enc.write(0);
    delay(2000);

    if (previousAccuracy - currentAccuracy <= 0)
    {
        Serial.println("Final achieved accuracy:");
        Serial.println(previousAccuracy);
        motorBrake();
        while (1)
            ; // Halt execution.
    }
  }
}

void pid_tuning() {
  motor1.attach(PWM_PIN);
  enc.write(0); // Zero the encoder

  // Configure the PID controller
  servo.myPID->SetTunings(KP, KI, KD);
  servo.setPWMSkip(PWM_SKIP);
  servo.setAccuracy(ACCURACY);
  servo.setMaxPWM(PWM_MAX);

  Serial.println("Setpoint,Input,Output");

  while (true) {
    static unsigned long lastPrintTime = millis();

    // Run the PID loop
    servo.run();

    // Print PID values for tuning via Serial Plotter every 20ms
    if (millis() - lastPrintTime > 20)
    {
        lastPrintTime = millis();
        Serial.println(servo.getSerialPlotter());
    }

    // Allow PID tuning commands via Serial
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("KP="))
        {
            KP = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KP set to: ");
            Serial.println(KP);
        }
        else if (command.startsWith("KI="))
        {
            KI = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KI set to: ");
            Serial.println(KI);
        }
        else if (command.startsWith("KD="))
        {
            KD = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KD set to: ");
            Serial.println(KD);
        }
        else if (command.startsWith("MOVE="))
        {
            long newPosition = command.substring(5).toInt();
            if (newPosition != 0)
            {
                servo.move(newPosition);
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
                servo.setMaxPWM(maxPWM);
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