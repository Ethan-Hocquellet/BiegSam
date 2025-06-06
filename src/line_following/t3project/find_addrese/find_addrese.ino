#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("开始扫描 I2C 地址...");
  
  for (byte address = 8; address < 120; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("找到设备，地址：0x");
      Serial.println(address, HEX);
    }
  }
}

void loop() {
  // 不需要在 loop() 中做任何事情
}
