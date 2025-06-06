#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>


#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
// const char* ssid = "hajimi666";
// const char* password = "password";

#define LDR_PIN A0
WiFiUDP udp;
unsigned int localPort = 55500;
char packetBuffer[64];

MotoronI2C m1(0x10);
MotoronI2C m2(0x50);

const int buttonPin = 2;
bool lastButtonState = HIGH;
bool buttonPressed = false;

Servo myServo;
const int SERVO_PIN = 52;
const int SERVO_ANGLE = 150;
uint16_t leftThreshold = 0;
uint16_t frontThreshold = 0;
uint16_t rightThreshold = 0;
unsigned long leftDetectStart = 0;
unsigned long rightDetectStart = 0;
bool leftDetecting = false;
bool rightDetecting = false;
const unsigned long confirmDelay = 200;

MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

const uint8_t SensorCount = 12;
const uint8_t sensorPins[SensorCount] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
uint16_t threshold = 0;
bool lastDarkState = false; 
float Kp = 0.04;
float Ki = 0;
float Kd = 0;
int lastError = 0;
long integral = 0;
int baseSpeed = 250;
int maxTurnSpeed = 500;

int lastLeftSpeed = 0;
int lastRightSpeed = 0;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int linePosition = 0;
uint8_t blackLineCount = 0;
const unsigned long lostLineTimeout = 500;

enum State {
  FOLLOW_LINE,
  SMALL_TURN_LEFT,
  SMALL_TURN_RIGHT,
  BIG_TURN_LEFT,
  BIG_TURN_RIGHT,
  GO_STRAIGHT,
  LOST_LINE,
  END_OF_PATH,
  AWAIT_TURN_LEFT_CONFIRM,
  AWAIT_TURN_RIGHT_CONFIRM
};

State currentState = FOLLOW_LINE;

const char route[] = {'L'};
const uint8_t routeLength = sizeof(route) / sizeof(route[0]);
uint8_t routeIndex = 0;
bool pathPointTriggered = false;

void setMotor(int motor, int speed) {
  speed = constrain(speed, -800, 800);
  switch (motor) {
    case 1: mc2.setSpeed(1, speed); break;
    case 2: mc1.setSpeed(1, speed); break;
    case 3: mc2.setSpeed(3, -speed); break;
    case 4: mc1.setSpeed(3, speed); break;
  }
}
void setLeftMotors(int speed) { setMotor(2, -speed); }
void setRightMotors(int speed) { setMotor(4, -speed); }

uint8_t countBlackLineSegments() {
  uint8_t segments = 0;
  bool onLine = false;
  for (uint8_t i = 0; i < 9; i++) {
    if (sensorValues[i] > threshold) {
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
bool isDark(int threshold = 500) {
  return analogRead(LDR_PIN) < threshold;
}

void calibration() {
  int p = 1;
  Serial.println("Calibration started...");
  for (int i = 0; i < 225; i++) {
    waitForButtonPressBlocking();
    qtr.calibrate();
    setLeftMotors(-p * 200);
    setRightMotors(p * 200);
    delay(5);
    if (i % 25 == 0) p *= -1;
  }
  threshold = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    threshold += (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  }
  threshold /= SensorCount;
  leftThreshold  = (qtr.calibrationOn.minimum[9]  + qtr.calibrationOn.maximum[9])  / 2;
  frontThreshold = (qtr.calibrationOn.minimum[10] + qtr.calibrationOn.maximum[10]) / 2;
  rightThreshold = (qtr.calibrationOn.minimum[11] + qtr.calibrationOn.maximum[11]) / 2;
  Serial.print("Calibration complete. Threshold: ");
  Serial.println(threshold);

  while (countBlackLineSegments() == 0) {
    waitForButtonPressBlocking();
    setLeftMotors(-baseSpeed);
    setRightMotors(baseSpeed);
    qtr.read(sensorValues);
    delay(20);
  }
  setLeftMotors(0);
  setRightMotors(0);
}

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
  setLeftMotors(0);
  setRightMotors(0);
}

unsigned long lastLineTime = 0;
bool leftPendingTurn = false;
bool rightPendingTurn = false;

void updateState() {
  bool leftDetected = sensorValues[9] > leftThreshold;
  bool frontDetected = sensorValues[10] > frontThreshold;
  bool rightDetected = sensorValues[11] > rightThreshold;

  uint8_t activeCount = 0;
  if (leftDetected) activeCount++;
  if (frontDetected) activeCount++;
  if (rightDetected) activeCount++;

  blackLineCount = countBlackLineSegments();

  Serial.print("Sensors: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]); Serial.print(" ");
  }
  Serial.print("| L:"); Serial.print(leftDetected);
  Serial.print(" F:"); Serial.print(frontDetected);
  Serial.print(" R:"); Serial.print(rightDetected);
  Serial.print(" | State:"); Serial.print(currentState);
  Serial.print(" | RouteIndex:"); Serial.print(routeIndex);
  Serial.print(" | Segments:"); Serial.print(blackLineCount);
  Serial.print(" Pos:"); Serial.print(linePosition);
  Serial.print(" | Lspd:"); Serial.print(currentLeftSpeed);
  Serial.print(" Rspd:"); Serial.println(currentRightSpeed);

  if (!pathPointTriggered && activeCount >= 2 && routeIndex < routeLength) {
    char currentOrder = route[routeIndex++];
    pathPointTriggered = true;
    switch (currentOrder) {
      case 'S': currentState = GO_STRAIGHT; break;
      case 'L': currentState = SMALL_TURN_LEFT; break;
      case 'R': currentState = SMALL_TURN_RIGHT; break;
      case 'A': currentState = BIG_TURN_LEFT; break;
      case 'D': currentState = BIG_TURN_RIGHT; break;
      default: currentState = FOLLOW_LINE; break;
    }
    Serial.print("Path point detected. Command: ");
    Serial.println(currentOrder);
    return;
  }

  if (!pathPointTriggered) {
    if (leftDetected && !leftPendingTurn) {
      leftPendingTurn = true;
      Serial.println("Left side detected — pending confirmation.");
    }
    if (rightDetected && !rightPendingTurn) {
      rightPendingTurn = true;
      Serial.println("Right side detected — pending confirmation.");
    }
  }

  if (!pathPointTriggered && blackLineCount == 0) {
    if (leftPendingTurn) {
      currentState = AWAIT_TURN_LEFT_CONFIRM;
      pathPointTriggered = true;
      leftPendingTurn = false;
      rightPendingTurn = false;
      Serial.println("Awaiting BIG LEFT turn confirmation.");
      return;
    }
    if (rightPendingTurn) {
      currentState = AWAIT_TURN_RIGHT_CONFIRM;
      pathPointTriggered = true;
      leftPendingTurn = false;
      rightPendingTurn = false;
      Serial.println("Awaiting BIG RIGHT turn confirmation.");
      return;
    }

    if (millis() - lastLineTime < lostLineTimeout) {
      currentState = LOST_LINE;
    } else {
      currentState = END_OF_PATH;
    }
    return;
  }

  if (!pathPointTriggered) {
    currentState = FOLLOW_LINE;
    lastLineTime = millis();
    leftPendingTurn = false;
    rightPendingTurn = false;
  }
}

void waitFrontSensorReacquire() {
  while (sensorValues[10] > frontThreshold){ 
    qtr.read(sensorValues);
    waitForButtonPressBlocking();
  }
  while (sensorValues[10] < frontThreshold){
     qtr.read(sensorValues);
     waitForButtonPressBlocking();
  }
}

void handleState() {
  switch (currentState) {
    case FOLLOW_LINE: {
      linePosition = qtr.readLineBlack(sensorValues);
      int error = linePosition - 4000;
      integral += error;
      integral = constrain(integral, -500, 500);
      int derivative = error - lastError;
      int powerDiff = Kp * error + Ki * integral + Kd * derivative;

      currentLeftSpeed = baseSpeed + powerDiff;
      currentRightSpeed = baseSpeed - powerDiff;

      setLeftMotors(currentLeftSpeed);
      setRightMotors(currentRightSpeed);

      lastLeftSpeed = currentLeftSpeed;
      lastRightSpeed = currentRightSpeed;
      lastError = error;
      pathPointTriggered = false;
      break;
    }
    case SMALL_TURN_LEFT:
      setLeftMotors(0);
      setRightMotors(300);
      waitFrontSensorReacquire();
      currentState = FOLLOW_LINE;
      break;
    case SMALL_TURN_RIGHT:
      setLeftMotors(300);
      setRightMotors(0);
      waitFrontSensorReacquire();
      currentState = FOLLOW_LINE;
      break;
    case BIG_TURN_LEFT:
      setLeftMotors(-maxTurnSpeed);
      setRightMotors(maxTurnSpeed);
      waitFrontSensorReacquire();
      setLeftMotors(0);
      setRightMotors(0);
      delay(500);
      currentState = FOLLOW_LINE;
      break;
    case BIG_TURN_RIGHT:
      setLeftMotors(maxTurnSpeed);
      setRightMotors(-maxTurnSpeed);
      waitFrontSensorReacquire();
      setLeftMotors(0);
      setRightMotors(0);
      delay(500);
      currentState = FOLLOW_LINE;
      break;
    case GO_STRAIGHT:
      setLeftMotors(baseSpeed);
      setRightMotors(baseSpeed);
      delay(400);
      currentState = FOLLOW_LINE;
      break;
    case LOST_LINE:
      setLeftMotors(lastLeftSpeed);
      setRightMotors(lastRightSpeed);
      break;
    case END_OF_PATH:
      setLeftMotors(0);
      setRightMotors(0);
      Serial.println("End of path reached!");
      break;
        case AWAIT_TURN_LEFT_CONFIRM:
    case AWAIT_TURN_RIGHT_CONFIRM: {
      bool leftStillOn = sensorValues[9] > leftThreshold;
      bool rightStillOn = sensorValues[11] > rightThreshold;

      Serial.print("Awaiting Turn | Left On: ");
      Serial.print(leftStillOn);
      Serial.print(" Right On: ");
      Serial.println(rightStillOn);

      if (!leftStillOn && !rightStillOn) {
        currentState = (currentState == AWAIT_TURN_LEFT_CONFIRM) ? BIG_TURN_LEFT : BIG_TURN_RIGHT;
        Serial.println("Both sides off-line, proceeding to BIG TURN.");
      } else {
        currentState = FOLLOW_LINE;
        pathPointTriggered = false; // 重新允许触发
        Serial.println("Side still on line, canceling turn and continuing PID.");
      }
      break;
    }

  }
}
void killswitchsetup(){
  // pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(buttonPin, INPUT_PULLUP);

  // WiFi.begin(ssid, password);

  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // Serial.println("\nWi-Fi connected.");
  // Serial.print("Local IP address: ");
  // Serial.println(WiFi.localIP());

  // udp.begin(localPort);
}
void setup() {
  Serial.begin(115200);
  MotoronSetup();
  killswitchsetup();
  myServo.attach(SERVO_PIN);
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
  lastLineTime = millis();
}
void recali() {
  bool currentDark = isDark();
  // 检测状态是否变化
  if (currentDark != lastDarkState) {
    calibration();  // 状态发生变化时，执行校准
    lastDarkState = currentDark;  // 更新记录状态
  }
}
void loop() {
  qtr.read(sensorValues);
  recali();
  updateState();
  handleState();
  waitForButtonPressBlocking();
  delay(10);
}
void waitForButtonPressBlocking() {
  // bool localButtonPressed = false;
  // bool localLastButtonState = lastButtonState; // 用你的 lastButtonState 变量初值
  // int i = 0;
  // while (!localButtonPressed) {
  //   // 保持 UDP 消息监听
  //   if(i==1){
  //     setLeftMotors(0);
  //     setRightMotors(0);
  //   }
  //   i = 1;
  //   int packetSize = udp.parsePacket();
  //   if (packetSize) {
  //     int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
  //     if (len > 0) packetBuffer[len] = '\0';

  //     Serial.print("Received UDP message: ");
  //     Serial.println(packetBuffer);

  //     if (strcmp(packetBuffer, "Stop") == 0) {
  //       Serial.println("Received Stop command (in wait loop)");
  //     }
  //   }

  //   // 按钮状态检测
  //   bool currentState = digitalRead(buttonPin);
  //   if (localLastButtonState == HIGH && currentState == LOW) {
  //     localButtonPressed = true;  // 检测到按钮按下，退出阻塞
  //   }
  //   localLastButtonState = currentState;

  //   delay(10); // 防抖和减负
    
  // }
  // // 同步外部 lastButtonState，保持状态一致
  // lastButtonState = localLastButtonState;
}
