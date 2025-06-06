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

// ======= PID 参数 =======
float Kp = 0.04;
float Ki = 0;
float Kd = 0;
int lastError = 0;
long integral = 0;
int baseSpeed = 250;
//
WiFiUDP udp;
unsigned int localPort = 55500;
char packetBuffer[64];
const int buttonPin = 2;
bool lastButtonState = HIGH;
bool buttonPressed = false;
Servo myservo;

// ======= 硬件配置 =======
MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

const uint8_t SensorCount = 9;
const uint8_t sensorPins[SensorCount] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
QTRSensors qtr;
uint16_t sensorValues[SensorCount];


void waitForButtonPressBlocking() {
  bool localButtonPressed = false;
  bool localLastButtonState = lastButtonState; // 用你的 lastButtonState 变量初值
  int i = 0;
  while (!localButtonPressed) {
    // 保持 UDP 消息监听
    if(i==1){
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
        // 这里不调用stopAllMotors，按你要求不控制马达
        Serial.println("Received Stop command (in wait loop)");
      }
    }

    // 按钮状态检测
    bool currentState = digitalRead(buttonPin);
    if (localLastButtonState == HIGH && currentState == LOW) {
      localButtonPressed = true;  // 检测到按钮按下，退出阻塞
    }
    localLastButtonState = currentState;

    delay(10); // 防抖和减负
  }
  // 同步外部 lastButtonState，保持状态一致
  lastButtonState = localLastButtonState;
}
void killswitchsetup(){
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
  setMotor(2, -speed);
}

void setRightMotors(int speed) {
  setMotor(4, -speed);
}

// ======= 校准 =======
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
}

// ======= Arduino标准函数 =======
void setup() {
  Serial.begin(115200);
  MotoronSetup();
  myservo.attach(52);
  int pos = 0;
  Serial.println("两个 Motoron 初始化完成。");
  for (pos = 30; pos <= 140; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  delay(5000);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  setLeftMotors(0);
  setRightMotors(0);
  calibration();
}

void loop() {
  // 读取传感器
  int position = qtr.readLineBlack(sensorValues);

  // 打印传感器值
  Serial.print("Sensors: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.print(" | Position: ");
  Serial.println(position);

  // PID
  int error = position - 4000;  // 期望位置是4000（中心）
  integral += error;
  integral = constrain(integral, -500, 500);
  int derivative = error - lastError;
  int powerDifference = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + powerDifference;
  int rightSpeed = baseSpeed - powerDifference;
  waitForButtonPressBlocking();
  setLeftMotors(leftSpeed);
  setRightMotors(rightSpeed);

  lastError = error;
  delay(20);
}


