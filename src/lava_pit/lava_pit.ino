#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>

// ========== Hardware Configuration ==========
MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

// ========== Servo Pins ==========
const int servoPin = 52;
const int hookPin = 53;
Servo servo;
Servo hook;

// ========== Motor Control ==========
void setMotor(int motor, int speed) {
  speed = constrain(speed, -800, 800);
  switch (motor) {
    case 1: mc2.setSpeed(1, -speed); break;
    case 2: mc1.setSpeed(1, speed); break;
    case 3: mc2.setSpeed(3, -speed); break;
    case 4: mc1.setSpeed(3, -speed); break;
  }
}
void setLeftMotors(int speed) { setMotor(2, speed);setMotor(1, -speed); }
void setRightMotors(int speed) { setMotor(4, speed);setMotor(3, -speed); }

// ========== Motoron Setup ==========
void MotoronSetup() {
  Wire.begin();
  mc1.setBus(&Wire);
  mc2.setBus(&Wire);
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.disableCommandTimeout();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.disableCommandTimeout();
  mc2.clearResetFlag();
}

void setup() {
  Serial.begin(115200);

  // Initialize Motoron controllers
  MotoronSetup();
  setLeftMotors(0);
  setRightMotors(0);

  // Initialize servos
  servo.attach(servoPin);
  hook.attach(hookPin);

  // 1️⃣ Move servo to 25 degrees
  servo.write(25);
  Serial.println("Servo set to 25°");
  delay(500);  // Give servo time to move

  // 2️⃣ Smoothly move hook from 50° to 0° in 0.5 seconds
  int startAngle = 50;
  int endAngle = 0;
  int steps = 10;                      // Divide into 10 steps
  float delayPerStep = 150 / steps;  // 0.5 seconds / 10 steps = 50ms

  for (int angle = startAngle; angle >= endAngle; angle -= (startAngle - endAngle) / steps) {
    hook.write(angle);
    delay(delayPerStep);
  }
  hook.write(endAngle);  // Ensure final angle is accurate
  Serial.println("Hook moved smoothly from 50° to 0°.");

  // 3️⃣ Wait for 2 seconds
  delay(2000);

  // 4️⃣ Full-speed backward
  setLeftMotors(-600);
  setRightMotors(-600);
  Serial.println("Full-speed backward started!");
}

void loop() {
  // Nothing here for now
}
