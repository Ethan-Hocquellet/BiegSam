#include <Wire.h>
#include <Motoron.h>

// ==================== 马达控制配置 ====================
MotoronI2C mc1(0x10);  // 控制后轮
MotoronI2C mc2(0x50);  // 控制前轮

const int dir_lf = -1;   // 左前轮 mc2(3)
const int dir_rf =  1;   // 右前轮 mc2(2)
const int dir_lr =  -1;   // 左后轮 mc1(3)
const int dir_rr = 1;   // 右后轮 mc1(2)
// ==================== 传感器配置 ====================
const int LEFT_SIDE_SENSOR = 49;
const int RIGHT_SIDE_SENSOR = 48;
const int SIDE_THRESHOLD = 2500;
const unsigned long DEBOUNCE_DELAY = 300;
// ==================== QTR RC读取实现 ====================
const int sensorPins[11] = {30, 31, 32, 33, 34, 35, 36, 37, 39, 49, 48};
unsigned int sensorValues[11];
unsigned int sensorMin[11];
unsigned int sensorMax[11];

// 原始读取，不归一化
void readRawSensors() {
  for (int i = 0; i < 11; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  for (int i = 0; i < 11; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  unsigned long startTime = micros();
  unsigned int timeout = 5000;
  bool active[11];
  for (int i = 0; i < 11; i++) active[i] = true;

  while ((micros() - startTime) < timeout) {
    unsigned long elapsed = micros() - startTime;
    for (int i = 0; i < 11; i++) {
      if (active[i] && digitalRead(sensorPins[i]) == LOW) {
        sensorValues[i] = elapsed;
        active[i] = false;
      }
    }
  }

  for (int i = 0; i < 11; i++) {
    if (active[i]) sensorValues[i] = timeout;
  }
}

// 校准：记录黑白背景下每个传感器的最小最大值
void calibrateSensors(int whiteSamples = 100, int blackSamples = 100) {
  // 初始化最小最大值
  for (int i = 0; i < 11; i++) {
    sensorMin[i] = 5000;
    sensorMax[i] = 0;
  }

  Serial.println("== 放置在白色背景上，开始采样 ==");
  delay(2000);
  for (int t = 0; t < whiteSamples; t++) {
    readRawSensors();
    for (int i = 0; i < 11; i++) {
      if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
    }
    delay(5);
  }

  Serial.println("== 放置在黑线上，开始采样 ==");
  delay(2000);
  for (int t = 0; t < blackSamples; t++) {
    readRawSensors();
    for (int i = 0; i < 11; i++) {
      if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
    }
    delay(5);
  }

  Serial.println("== 校准完成：");
  for (int i = 0; i < 11; i++) {
    Serial.print("S"); Serial.print(i); Serial.print(": ");
    Serial.print(sensorMin[i]); Serial.print(" ~ ");
    Serial.println(sensorMax[i]);
  }
}

// 读取归一化后的传感器值：0=白，1000=黑
void readSensors() {
  readRawSensors();
  for (int i = 0; i < 11; i++) {
    sensorValues[i] = constrain(sensorValues[i], sensorMin[i], sensorMax[i]);
    sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
  }
}

// 返回线的位置：0~8000，-1表示未检测到
int readLinePosition(bool invert = false) {
  readSensors();
  long weightedSum = 0;
  int valueSum = 0;

  for (int i = 0; i < 9; i++) {  // 仅中间9个用于定位
    uint16_t val = sensorValues[i];
    if (invert) val = 1000 - val;

    if (val > 200) {  // 忽略太白的区域
      weightedSum += (long)val * i * 1000;
      valueSum += val;
    }
  }

  return valueSum > 0 ? weightedSum / valueSum : -1;
}

// ==================== 路径规划 ====================
enum TurnCommand { CMD_CONTINUE, CMD_LEFT_90, CMD_RIGHT_90, CMD_STOP };

const TurnCommand pathSequence[] = {
  CMD_RIGHT_90, CMD_RIGHT_90, CMD_LEFT_90, 
  CMD_RIGHT_90, CMD_CONTINUE, CMD_LEFT_90,
  CMD_LEFT_90, CMD_LEFT_90, CMD_CONTINUE,
  CMD_LEFT_90, CMD_LEFT_90, CMD_STOP
};

// ==================== PID 控制器 ====================
class PID {
public:
  PID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

  void reset() {
    _lastError = 0;
    _integral = 0;
  }

  int compute(int error) {
    unsigned long now = millis();
    float deltaTime = (now - _lastTime) / 1000.0;
    _lastTime = now;

    _integral += error * deltaTime;
    float derivative = (error - _lastError) / deltaTime;

    float output = _kp * error + _ki * _integral + _kd * derivative;
    _lastError = error;

    return (int)output;
  }

private:
  float _kp, _ki, _kd;
  float _integral = 0;
  int _lastError = 0;
  unsigned long _lastTime = 0;
};

// ==================== 控制逻辑类 ====================
class LineFollower {
public:
  void setup() {
    Serial.begin(9600);
    Wire.begin();
    calibrateSensors();
    mc1.reinitialize();
    mc1.disableCrc();
    mc1.disableCommandTimeout();
    mc1.clearResetFlag();

    mc2.reinitialize();
    mc2.disableCrc();
    mc2.disableCommandTimeout();
    mc2.clearResetFlag();

    Serial.println("系统初始化完成。");
  }

  void loop() {
    int triggerType = checkTrigger();
    
    switch (_state) {
      case FOLLOWING:
        followLine();
        if (triggerType > 0 && !_executingCommand) {
          handleCommand(triggerType);
        }
        break;

      case TURNING:
        if (checkTurnComplete()) {
          completeTurn();
        }
        break;

      case STOPPED:
        stopMotors();
        break;
    }

    debugOutput();
    delay(10);
  }

private:
  enum State { FOLLOWING, TURNING, STOPPED };
  State _state = FOLLOWING;
  uint8_t _currentCmdIndex = 0;
  bool _executingCommand = false;
  unsigned long _lastTriggerTime = 0;
  int _lastTriggerType = 0;
  unsigned long _turnStartTime = 0;
  int _turnDirection = 0;


  PID _linePid = PID(10, 0.0, 0.05);     // 巡线 PID
  PID _turnPid = PID(0.2, 0.0, 0.01);     // 转弯 PID（稍微响应更快）

  void setMotors(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    mc1.setSpeed(3, dir_lr * left);   // 左后
    mc1.setSpeed(2, dir_rr * right);  // 右后
    mc2.setSpeed(3, dir_lf * left);   // 左前
    mc2.setSpeed(2, dir_rf * right);  // 右前
  }

  void stopMotors() { setMotors(0, 0); }

  int checkTrigger() {
    if (millis() - _lastTriggerTime < DEBOUNCE_DELAY) return 0;

    readSensors();
    int leftVal = sensorValues[9]; 
    int rightVal = sensorValues[10]; 

    int segments = 0;
    bool lastBlack = false;
    for (int i = 0; i < 9; i++) {
      if (sensorValues[i] > 1000) {
        if (!lastBlack) segments++;
        lastBlack = true;
      } else {
        lastBlack = false;
      }
    }

    if (segments > 1) return 3;
    if (leftVal < SIDE_THRESHOLD && rightVal < SIDE_THRESHOLD) return random(1, 3);
    if (leftVal < SIDE_THRESHOLD) return 1;
    if (rightVal < SIDE_THRESHOLD) return 2;
    return 0;
  }

  void followLine() {
    int position = readLinePosition(true);
    if (position < 0) {
      setMotors(50, -50);  // 原地找线
      return;
    }

    int error = position - 4000;
    int correction = _linePid.compute(error);

    int baseSpeed = 150;
    setMotors(baseSpeed + correction, baseSpeed - correction);
  }

  void handleCommand(int triggerType) {
    _executingCommand = true;
    _lastTriggerType = triggerType;
    _lastTriggerTime = millis();

    TurnCommand cmd = pathSequence[_currentCmdIndex];
    switch (cmd) {
      case CMD_LEFT_90:
        startTurn(-100, 100);
        break;
      case CMD_RIGHT_90:
        startTurn(100, -100);
        break;
      case CMD_STOP:
        _state = STOPPED;
        break;
      default:
        _currentCmdIndex++;
        _executingCommand = false;
    }
  }

  void startTurn(int leftSpeed, int rightSpeed) {
    _state = TURNING;
    _turnStartTime = millis();
    _turnDirection = (leftSpeed < 0) ? -1 : 1;  // -1 左转，1 右转
    _turnPid.reset();
  }

  void pidTurnControl() {
    int position = readLinePosition(true);
    if (position < 0) {
      setMotors(_turnDirection * 100, -_turnDirection * 100);
      return;
    }

    int error = position - 4000;
    int pidOutput = _turnPid.compute(error);

    int outerSpeed = 100;
    int innerSpeed = constrain(pidOutput, -100, 100);

    if (_turnDirection == 1) { // 右转
      setMotors(-innerSpeed, outerSpeed);
    } else { // 左转
      setMotors(outerSpeed, -innerSpeed);
    }
  }

  bool checkTurnComplete() {
    if (_lastTriggerType == 3) {
      readSensors();
      int segments = 0;
      for (int i = 0; i < 9; i++) {
        if (sensorValues[i] > 1000) segments++;
      }
      return segments <= 1;
    } else if (_lastTriggerType == 1) {
      return analogRead(LEFT_SIDE_SENSOR) > SIDE_THRESHOLD;
    } else if (_lastTriggerType == 2) {
      return analogRead(RIGHT_SIDE_SENSOR) > SIDE_THRESHOLD;
    }

    return (millis() - _turnStartTime) > 2000;
  }

  void completeTurn() {
    _currentCmdIndex++;
    _executingCommand = false;
    _state = FOLLOWING;
    stopMotors();
    delay(100);
  }

  void debugOutput() {
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime < 500) return;

    Serial.print("State:");
    Serial.print(_state);
    Serial.print(" Cmd:");
    Serial.print(_currentCmdIndex);
    Serial.print(" Trig:");
    Serial.print(_lastTriggerType);
    Serial.print(" Sensors:");
    for (int i = 0; i < 9; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(",");
    }
    Serial.print("left:");
    Serial.print(sensorValues[9]);
    Serial.print("right:");
    Serial.print(sensorValues[10]);
    Serial.println();



    lastDebugTime = millis();
  }
};

// ==================== 主程序入口 ====================
LineFollower follower;

void setup() {
  follower.setup();

}

void loop() {
  follower.loop();
}
