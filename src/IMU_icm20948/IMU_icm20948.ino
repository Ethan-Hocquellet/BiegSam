#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;               // Current accumulated angle (around Z axis)
unsigned long lastTime = 0;      // Last read timestamp (milliseconds)

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
  myIMU.autoOffsets();  // Automatically calibrate the gyroscope
  Serial.println("Done!");

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  lastTime = millis();  // Initialize time
}

void loop() {
  myIMU.readSensor();
  xyzFloat gyr;
  myIMU.getGyrValues(&gyr);  // Get angular velocity (unit: °/s)

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Calculate time difference, unit: seconds
  lastTime = currentTime;

  // Filtering
  float threshold = 0.2;
  if (abs(gyr.x) < threshold) gyr.x = 0.0;
  if (abs(gyr.y) < threshold) gyr.y = 0.0;
  if (abs(gyr.z) < threshold) gyr.z = 0.0;

  // Accumulate angle (around Z axis): angular velocity × time difference
  angleX += gyr.x * dt;
  angleY += gyr.y * dt;
  angleZ += gyr.z * dt;

  // Output current angle information

  // Serial.print("Angular velocity (°/s): X=");
  // Serial.print(gyr.x);
  // Serial.print(" Y=");
  // Serial.print(gyr.y);
  // Serial.print(" Z=");
  // Serial.println(gyr.z);

  Serial.print("Accumulated angle (°): X=");
  Serial.print(angleX);
  Serial.print(" Y=");
  Serial.print(angleY);
  Serial.print(" Z=");
  Serial.println(angleZ);

  delay(20); // Approx. 50Hz sampling
}
