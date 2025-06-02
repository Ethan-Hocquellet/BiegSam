#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>

Servo myservo; 

// ======= 硬件配置 =======
MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

const uint8_t SensorCount = 12;
const uint8_t sensorPins[SensorCount] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
uint16_t threshold = 0;

// ======= PID 参数 =======
float Kp = 0.04;
float Ki = 0;
float Kd = 0;
int lastError = 0;
long integral = 0;
int baseSpeed = 200;

// ======= 路口 & 状态管理 =======
unsigned long lastLineTime = 0;
bool isTurningBack = false;

enum TurnDirection {
  LEFT,
  STRAIGHT,
  RIGHT
};

TurnDirection turnPriority[3] = {LEFT, STRAIGHT, RIGHT};

enum PathState {
  FOLLOW_LINE,
  AT_TURN_LEFT,
  AT_TURN_RIGHT,
  GO_STRAIGHT,
  END_OF_PATH,
  T90_DEGREE_LEFT,
  T90_DEGREE_RIGHT
};

PathState currentState = FOLLOW_LINE;
int waypointCounter = 0;

// ======= 电机控制 =======
void setMotor(int motor, int speed) {
  speed = constrain(speed, -800, 800);
  switch (motor) {
    case 1: mc2.setSpeed(1, speed); break;
    case 2: mc1.setSpeed(1, speed); break;
    case 3: mc2.setSpeed(3, -speed); break;
    case 4: mc1.setSpeed(3, speed); break;
  }
}

void setLeftMotors(int speed) {
  setMotor(2, speed);

}

void setRightMotors(int speed) {
  setMotor(4, speed);
}

// ======= 传感器辅助 =======
uint8_t countBlackLineSegments() {
  uint8_t segments = 0;
  bool onLine = false;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < threshold) {
      if (!onLine) {
        segments++;
        onLine = true;
      }
    } else {
      onLine = false;
    }
  }
  return segments;
}

// ======= 按优先顺序转向决策 =======
PathState decideTurnByPriority(bool left, bool front, bool right) {
  for (int i = 0; i < 3; i++) {
    TurnDirection dir = turnPriority[i];
    if (dir == LEFT && left) return AT_TURN_LEFT;
    if (dir == STRAIGHT && front) return GO_STRAIGHT;
    if (dir == RIGHT && right) return AT_TURN_RIGHT;
  }
  return END_OF_PATH;
}

// ======= 校准 =======
void calibration() {
  int p = 1;
  Serial.println("Calibrating...");
  for (int i = 0; i < 225; i++) {
    qtr.calibrate();
    setLeftMotors(-p * 100);
    setRightMotors(p * 100);
    delay(10);
    if (i % 50 == 0) p *= -1;
  }
  threshold = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    threshold += (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  }
  threshold /= SensorCount;
  Serial.print("Threshold: ");
  Serial.println(threshold);
}

// ======= Motoron初始化 =======
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

// ======= 路口状态更新 =======
void updateState() {
  bool leftDetected = sensorValues[9] > threshold;
  bool rightDetected = sensorValues[10] > threshold;
  bool frontDetected = sensorValues[11] > threshold;

  if (countBlackLineSegments() > 0) {
    lastLineTime = millis();
  }

  if (!isTurningBack && (millis() - lastLineTime > 2000)) {
    currentState = END_OF_PATH;
    return;
  }

  int detectedCount = 0;
  if (leftDetected) detectedCount++;
  if (rightDetected) detectedCount++;
  if (frontDetected) detectedCount++;

  if (detectedCount >= 2) {
    waypointCounter++;
    currentState = decideTurnByPriority(leftDetected, frontDetected, rightDetected);
    Serial.print("Crossroad. Decided to: ");
    Serial.println(
      (currentState == AT_TURN_LEFT) ? "Left" :
      (currentState == GO_STRAIGHT) ? "Straight" : "Right"
    );
  } else if (detectedCount == 1 && !frontDetected) {
    currentState = leftDetected ? T90_DEGREE_LEFT : T90_DEGREE_RIGHT;
    Serial.print("90-degree turn: ");
    Serial.println(leftDetected ? "Left" : "Right");
  } else {
    currentState = FOLLOW_LINE;
  }
}

// ======= 状态机执行 =======
void handleState() {
  switch (currentState) {
    case FOLLOW_LINE: {
      int position = qtr.readLineBlack(sensorValues);
      int error = position - 4000;
      integral += error;
      integral = constrain(integral, -500, 500);
      int derivative = error - lastError;
      int powerDifference = Kp * error + Ki * integral + Kd * derivative;

      int leftSpeed = baseSpeed + powerDifference;
      int rightSpeed = baseSpeed - powerDifference;

      setLeftMotors(leftSpeed);
      setRightMotors(rightSpeed);

      lastError = error;
      break;
    }

    case AT_TURN_LEFT:
      setLeftMotors(-80);
      setRightMotors(80);
      delay(400);
      currentState = FOLLOW_LINE;
      break;

    case AT_TURN_RIGHT:
      setLeftMotors(80);
      setRightMotors(-80);
      delay(400);
      currentState = FOLLOW_LINE;
      break;

    case GO_STRAIGHT:
      setLeftMotors(baseSpeed);
      setRightMotors(baseSpeed);
      delay(400);
      currentState = FOLLOW_LINE;
      break;

    case T90_DEGREE_LEFT:
      setLeftMotors(-100);
      setRightMotors(100);
      delay(600);
      currentState = FOLLOW_LINE;
      break;

    case T90_DEGREE_RIGHT:
      setLeftMotors(100);
      setRightMotors(-100);
      delay(600);
      currentState = FOLLOW_LINE;
      break;

    case END_OF_PATH:
      isTurningBack = true;
      Serial.println("Lost line! Turning back...");
      setLeftMotors(-150);
      setRightMotors(150);
      delay(500);
      setLeftMotors(0);
      setRightMotors(0);

      while (countBlackLineSegments() == 0) {
        setLeftMotors(baseSpeed);
        setRightMotors(baseSpeed);
        qtr.read(sensorValues);
        delay(20);
      }

      waypointCounter = max(0, waypointCounter - 1);
      currentState = FOLLOW_LINE;
      isTurningBack = false;
      lastLineTime = millis();
      Serial.println("Back on line!");
      break;
  }
}

// ======= Arduino标准函数 =======
void setup() {
  Serial.begin(115200);
  MotoronSetup();
  myservo.attach(52);
  int pos = 0;
  Serial.println("两个 Motoron 初始化完成。");
    for (pos = 30; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  delay(5000);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  setLeftMotors(0);
  setRightMotors(0);
  calibration();
  lastLineTime = millis();
  
}

void loop() {
  qtr.read(sensorValues);
  updateState();
  handleState();
  delay(20);
}
