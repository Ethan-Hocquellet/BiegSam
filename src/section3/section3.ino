#include <Wire.h>
#include <Servo.h>
#include <ICM20948_WE.h>
#include <QTRSensors.h>

// -------- Pin Definitions --------
const int DIST_SENSOR_PIN = A0;
const int HOOK_SERVO_PIN = 53;
const int SERVO_PIN = 52;
const int SERVO_ANGLE = 150;

Servo hookServo;
Servo myServo;

QTRSensors qtr;
const uint8_t SensorCount = 12;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[SensorCount] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int leftThreshold = 500;   // Adjust as needed
const int rightThreshold = 500;  // Adjust as needed

bool lavaTriggered = false;
bool lavaCompleted = false;
unsigned long lavaStartTime = 0;
bool returningFromLava = false;
bool droppedObject = false;
bool finalReturn = false;

// -------- Function Prototypes (External dependencies) --------
void setLeftMotors(int speed);
void setRightMotors(int speed);
void MotoronSetup();
void initIMU();
void updateIMU();
float getAngleZ();
void updateState();
void handleState();
void calibration();

// -------- Lava Pit Distance Read --------
float readLavaDistanceCM() {
  int raw = analogRead(DIST_SENSOR_PIN);
  float voltage = raw * (5.0 / 1023.0);
  float d = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2)
            - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);
  return d;
}

// -------- LavaPit State Machine --------
void handleLavaPit() {
  static unsigned long lastPhaseTime = 0;
  static int phase = 0;
  float angleZNow = getAngleZ();

  switch (phase) {
    case 0:
      setLeftMotors(-250);
      setRightMotors(-250);
      lastPhaseTime = millis();
      phase = 1;
      break;

    case 1:
      if (millis() - lastPhaseTime > 2000) {
        lastPhaseTime = millis();
        angleZNow = 0.0;
        phase = 2;
      }
      break;

    case 2:
      setLeftMotors(600);
      setRightMotors(-600);
      updateIMU();
      if (abs(getAngleZ()) >= 170.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 3;
      }
      break;

    case 3:
      myServo.write(25);
      delay(500);
      phase = 4;
      break;

    case 4:
      setLeftMotors(-400);
      setRightMotors(-400);
      if (readLavaDistanceCM() < 10.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 5;
      }
      break;

    case 5:
      hookServo.write(0);
      delay(1000);
      phase = 6;
      break;

    case 6:
      setLeftMotors(-650);
      setRightMotors(-650);
      if (readLavaDistanceCM() > 10.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 7;
      }
      break;

    case 7:
      hookServo.write(50);
      delay(1000);
      phase = 8;
      break;

    case 8:
      myServo.write(150);
      delay(500);
      phase = 9;
      angleZNow = 0.0;
      break;

    case 9:
      setLeftMotors(600);
      setRightMotors(-600);
      updateIMU();
      if (abs(getAngleZ()) >= 170.0) {
        setLeftMotors(0);
        setRightMotors(0);
        lavaCompleted = true;
        lavaTriggered = false;
        lavaStartTime = millis();
        phase = 0;
      }
      break;
  }
}

// -------- Main Loop --------
void loop() {
  updateIMU();
  qtr.read(sensorValues);

  if (!lavaTriggered && !lavaCompleted && readLavaDistanceCM() < 10.0) {
    lavaTriggered = true;
  }

  if (lavaTriggered && !lavaCompleted) {
    handleLavaPit();
    return;
  }

  if (lavaCompleted && !finalReturn) {
    updateState();
    handleState();

    if (millis() - lavaStartTime > 2000) {
      bool leftLine = sensorValues[9] > leftThreshold;
      bool rightLine = sensorValues[11] > rightThreshold;

      if (leftLine && rightLine) {
        setLeftMotors(800);
        setRightMotors(800);
        delay(1000);
        hookServo.write(0);
        delay(1000);
        finalReturn = true;
      }
    }
    return;
  }

  updateState();
  handleState();
  delay(10);
}

// -------- Setup --------
void setup() {
  Serial.begin(115200);
  MotoronSetup();
  initIMU();
  myServo.attach(SERVO_PIN);
  hookServo.attach(HOOK_SERVO_PIN);

  for (int angle = 25; angle <= SERVO_ANGLE; angle++) {
    myServo.write(angle);
    delay(25);
  }

  delay(3000);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  setLeftMotors(0);
  setRightMotors(0);
  calibration();
}
