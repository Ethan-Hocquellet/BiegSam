#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>

// ========== Hardware Configuration ==========
MotoronI2C mc1(0x10);   
MotoronI2C mc2(0x50);

// ========== Servo Pin ==========
const int servoPin = 52;
Servo servo;

// ========== Motor Control ==========
void setMotor(int motor, int speed) {
  speed = constrain(speed, -800, 800);
  switch (motor) {
    case 1: mc2.setSpeed(1, -speed); break;
    case 2: mc1.setSpeed(1, speed); break;
    case 3: mc2.setSpeed(3, -speed); break;
    case 4: mc1.setSpeed(3, speed); break;
  }
}
void setLeftMotors(int speed) { setMotor(2, -speed);setMotor(1, -speed); }
void setRightMotors(int speed) { setMotor(4, -speed);setMotor(3, -speed); }

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

  // Initialize servo
  servo.attach(servoPin);

  // 1️⃣ Move servo to 50 degrees
  servo.write(100);
  Serial.println("Servo set to 50°");
  
  // 2️⃣ Wait 1-2 seconds
  delay(1500);

  // 3️⃣ Full-speed backward
  setLeftMotors(-600);
  setRightMotors(-600);
  Serial.println("Full-speed backward started!");
}

void loop() {
  // Nothing here for now
}
