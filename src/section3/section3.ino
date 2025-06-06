// ------------- 变量和定义新增 -------------
const int DIST_SENSOR_PIN = A8; // 替换为你连接的模拟引脚
const int HOOK_SERVO_PIN = 53;
Servo hookServo;

bool lavaTriggered = false;
bool lavaCompleted = false;
unsigned long lavaStartTime = 0;
bool returningFromLava = false;
bool droppedObject = false;
bool finalReturn = false;

// ------------- 读取距离封装 -------------
float readLavaDistanceCM() {
  int raw = analogRead(DIST_SENSOR_PIN);
  float voltage = raw * (5.0 / 1023.0);
  float d = 33.9 - 69.5 * voltage + 62.3 * pow(voltage, 2)
            - 25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);
  return d;
}

// ------------- LavaPit 模式主逻辑 -------------
void handleLavaPit() {
  static unsigned long lastPhaseTime = 0;
  static int phase = 0;
  float angleZNow = getAngleZ();

  switch (phase) {
    case 0: // 倒车 2 秒
      setLeftMotors(-250);
      setRightMotors(-250);
      lastPhaseTime = millis();
      phase = 1;
      break;

    case 1: // 等 2 秒结束
      if (millis() - lastPhaseTime > 2000) {
        lastPhaseTime = millis();
        angleZ = 0.0; // 重置角度积分
        phase = 2;
      }
      break;

    case 2: // 180°原地掉头
      setLeftMotors(600);
      setRightMotors(-600);
      updateIMU();
      if (abs(getAngleZ()) >= 170.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 3;
      }
      break;

    case 3: // Servo 转到 25 度
      myServo.write(25);
      delay(500);
      phase = 4;
      break;

    case 4: // 倒车直到接近目标点
      setLeftMotors(-400);
      setRightMotors(-400);
      if (readLavaDistanceCM() < 10.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 5;
      }
      break;

    case 5: // 放下 hook = 0
      hookServo.write(0);
      delay(1000);
      phase = 6;
      break;

    case 6: // 以 650 速度倒车离开
      setLeftMotors(-650);
      setRightMotors(-650);
      if (readLavaDistanceCM() > 10.0) {
        setLeftMotors(0);
        setRightMotors(0);
        phase = 7;
      }
      break;

    case 7: // hook 抬起到 50，再停一秒
      hookServo.write(50);
      delay(1000);
      phase = 8;
      break;

    case 8: // Servo 恢复到 150
      myServo.write(150);
      delay(500);
      phase = 9;
      angleZ = 0.0;
      break;

    case 9: // 再次原地 180°旋转回来
      setLeftMotors(600);
      setRightMotors(-600);
      updateIMU();
      if (abs(getAngleZ()) >= 170.0) {
        setLeftMotors(0);
        setRightMotors(0);
        lavaCompleted = true;
        lavaTriggered = false;
        lavaStartTime = millis();
        phase = 0; // 重置状态机
      }
      break;
  }
}

// ------------- 更新主 loop() -------------
void loop() {
  updateIMU();
  qtr.read(sensorValues);

  if (!lavaTriggered && !lavaCompleted && readLavaDistanceCM() < 10.0) {
    lavaTriggered = true;
  }

  if (lavaTriggered && !lavaCompleted) {
    handleLavaPit();
    return; // 停止其他行为
  }

  // Lava 回归后的线循
  if (lavaCompleted && !finalReturn) {
    updateState();
    handleState();

    // 等两秒后，若左右都探测到线则触发最终动作
    if (millis() - lavaStartTime > 2000) {
      bool leftLine = sensorValues[9] > leftThreshold;
      bool rightLine = sensorValues[11] > rightThreshold;

      if (leftLine && rightLine) {
        setLeftMotors(800);
        setRightMotors(800);
        delay(1000);
        hookServo.write(0); // 放下钩子
        delay(1000);
        finalReturn = true; // 防止重复执行
      }
    }
    return;
  }

  // 初始普通行为
  updateState();
  handleState();
  delay(10);
}

// ------------- Setup 中新增 Hook Servo 初始化 -------------
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
