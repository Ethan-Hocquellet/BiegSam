#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;               // 当前累计角度（绕 Z 轴）
unsigned long lastTime = 0;      // 上一次读取的时间戳（毫秒）

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while(!Serial) {}

  if(!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();  // 自动校准陀螺仪
  Serial.println("Done!");

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  lastTime = millis();  // 初始化时间
}

void loop() {
  myIMU.readSensor();
  xyzFloat gyr;
  myIMU.getGyrValues(&gyr);  // 获取角速度（单位：°/s）

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // 计算时间差，单位：秒
  lastTime = currentTime;

  //滤波
  float threshold = 0.2;
  if (abs(gyr.x) < threshold) gyr.x = 0.0;
  if (abs(gyr.y) < threshold) gyr.y = 0.0;
  if (abs(gyr.z) < threshold) gyr.z = 0.0;

  // 累加角度（绕 Z 轴）：角速度 × 时间差
    angleX += gyr.x * dt;
  angleY += gyr.y * dt;
  angleZ += gyr.z * dt;

  // 输出当前角度信息

  // Serial.print("角速度 (°/s): X=");
  // Serial.print(gyr.x);
  // Serial.print(" Y=");
  // Serial.print(gyr.y);
  // Serial.print(" Z=");
  // Serial.println(gyr.z);

  Serial.print("累计角度 (°): X=");
  Serial.print(angleX);
  Serial.print(" Y=");
  Serial.print(angleY);
  Serial.print(" Z=");
  Serial.println(angleZ);

  delay(20); // 约50Hz采样
}
