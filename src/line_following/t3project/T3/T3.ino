#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";

float Kp = 0.04;
float Ki = 0;
float Kd = 0;
int lastError = 0;
long integral = 0;
int baseSpeed = 250;

WiFiUDP udp;
unsigned int localPort = 55500;
char packetBuffer[64];
const int buttonPin = 2;
bool lastButtonState = HIGH;

Servo myservo;

MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

const uint8_t SensorCount = 9;
const uint8_t sensorPins[SensorCount] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

int lastLeftSpeed = 0;
int lastRightSpeed = 0;
bool lineDetected = false;

void waitForButtonPressBlocking() {
  bool localButtonPressed = false;
  bool localLastButtonState = lastButtonState;
  int i = 0;
  while (!localButtonPressed) {
    if (i == 1) {
      setLeftMotors(0);
      setRightMotors(0);
    }
    i = 1;
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (len > 0) packetBuffer[len] = '\0';

      Serial.print("Received UDP message: ");
      Serial.println(packetBuffer);

      if (strcmp(packetBuffer, "Stop") == 0) {
        Serial.println("Received Stop command (in wait loop)");
      }
      if (strcmp(packetBuffer, "Start") == 0) {
        localButtonPressed = true;
      }
    }

    bool currentState = digitalRead(buttonPin);
    if (localLastButtonState == HIGH && currentState == LOW) {
      localButtonPressed = true;
    }
    localLastButtonState = currentState;

    delay(10);
  }
  lastButtonState = localLastButtonState;
}

void killswitchsetup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
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
}

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
  setMotor(2, -speed);
}

void setRightMotors(int speed) {
  setMotor(4, -speed);
}

void calibration() {
  int p = 1;
  Serial.println("Calibrating...");
  for (int i = 0; i < 225; i++) {
    waitForButtonPressBlocking();
    qtr.calibrate();
    setLeftMotors(-p * 200);
    setRightMotors(p * 200);
    delay(5);
    if (i % 25 == 0){
      p *= -1;
      setLeftMotors(0);
      setRightMotors(0);
      delay(500);
    }
  }
  Serial.println("Calibration complete.");
    islineDetected();
    while (lineDetected) {
    islineDetected();
    waitForButtonPressBlocking();
    setLeftMotors(-baseSpeed);
    setRightMotors(baseSpeed);
    qtr.read(sensorValues);
    delay(20);
  }
  setLeftMotors(0);
  setRightMotors(0);
}

void setup() {
  Serial.begin(115200);
  MotoronSetup();
  killswitchsetup();
  myservo.attach(52);
  Serial.println("两个 Motoron 初始化完成。");

  for (int pos = 30; pos <= 140; pos++) {
    myservo.write(pos);
    delay(15);
  }

  delay(5000);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  setLeftMotors(0);
  setRightMotors(0);
  calibration();
  waitForButtonPressBlocking();  // 启动前等待信号
}
void islineDetected(){
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 600) {  // 阈值可根据环境调
      lineDetected = true;
      break;
    }
  }
  return;
}
void loop() {
  // 读取传感器
  int position = qtr.readLineBlack(sensorValues);

  // 判断是否检测到线
  lineDetected = false;
  islineDetected();

  int leftSpeed = 0;
  int rightSpeed = 0;

  if (lineDetected) {
    int error = position - 4000;
    integral += error;
    integral = constrain(integral, -500, 500);
    int derivative = error - lastError;
    int powerDifference = Kp * error + Ki * integral + Kd * derivative;

    leftSpeed = baseSpeed + powerDifference;
    rightSpeed = baseSpeed - powerDifference;

    lastLeftSpeed = leftSpeed;
    lastRightSpeed = rightSpeed;
    lastError = error;
  } else {
    // 丢线，使用上次速度
    leftSpeed = lastLeftSpeed;
    rightSpeed = lastRightSpeed;
    Serial.println("Line lost! Using last known speed.");
  }

  setLeftMotors(leftSpeed);
  setRightMotors(rightSpeed);

  delay(20);
}
