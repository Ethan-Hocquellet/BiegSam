#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <ICM20948_WE.h>
#include <math.h>

#define AVERAGE_OF 50
#define MCU_VOLTAGE 5
#define ICM20948_ADDR 0x68 // ICM20948 address;
#define DEBUG 1 // Set to 1 to enable debug messages


Servo myservo;
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR); 
MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);

const int n = 2; // Number of IR Distance
const int u_n = 2; // Number of Ultrasonic
const int sensorPin[] = {A0, A1}; // n must match array size of sensorPin[]}
float distance[n];
float u_distance[u_n];
long duration[u_n]; // duration for two ultrasonic sensors
float u_duration[u_n];
float angleError, distanceError;
float angle[3]; // angle[0] = x, angle[1] = y, angle[2] = z

// turning
float startAngle = 0.0; // Starting angle for turning function
bool isTurning = false;

const float sensorSeparation = 23; // distance between the two IR distance sensors

// Ultrasonic 1
const int trigPin1 = 0;
const int echoPin1 = 1;
// Ultrasonic 2
const int trigPin2 = 11; // change for correct pins

const int echoPin2 = 12;

enum ServoMode { SHORT_MODE, LONG_MODE };
ServoMode servoMode = SHORT_MODE; // Default mode

// define wall following and turning modes
enum WallMode { WALL_FOLLOW, TURNING };
WallMode wallMode = WALL_FOLLOW;

enum TurnDirection { CLOCKWISE, ANTICLOCKWISE, FLIP, STRAIGHT };
TurnDirection turnDirection = CLOCKWISE; // Default turn direction

enum Orientation { LINE_TRACKING, WALL_TRACKING }; // LINE_TRACKING = black first
Orientation orientation = LINE_TRACKING; // Default orientation

// Func prototypes because Sophia told us to :)
void readDistance(int sensor);
// @readDistance for GP2Y0A51SK0F
void readUltrasonic(void);
// @readUltrasonic for Ultrasonic
void turnRight(void);
// @turnRight for turning right by 90 degrees :)
void PID(void);
// @PID for PID control of the robot
void setMotor(int motor, int speed);
// @setMotor for setting motor speed
void setLeftMotors(int speed);
// @setLeftMotors for setting left motors speed
void setRightMotors(int speed);
// @setRightMotors for setting right motors speed DEPRECATED
void setGreenLeft(int speed);
// @setGreenLeft for setting the green left motor speed
void setBlackLeft(int speed);
// @setBlackLeft for setting the black left motor speed
void setGreenRight(int speed);
// @setGreenRight for setting the green right motor speed
void setBlackRight(int speed);
// @setBlackRight for setting the black right motor speed
// @setRightMotors for setting right motors speed
void MotoronSetup(void);
// @MotoronSetup for initialising the Motoron I2C bus
void setServoMax(void);
// @setServoMax for setting the servo to maximum position
void setServoMin(void);
// @setServoMin for setting the servo to minimum position
void setServoFlat(void);
// @setServoFlat for setting the servo to flat position
void IMU(void);
// @IMU for reading the IMU data and calculating angles
void turnAngle(TurnDirection turnDirection);
// @turnAngle for turning the robot by a specified angle

// May need to add an alignment function e.g. void alignWall(void);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  MotoronSetup(); // Initialise Motors
  myservo.attach(52);

  // Ultrasonic sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);

  if (servoMode == SHORT_MODE) {
    setServoMax(); // Set servo to maximum position
    Serial.println("Servo set to maximum position.");
  } else if (servoMode == LONG_MODE) {
    setServoFlat(); // Set servo to minimum position
    Serial.println("Servo set to flat position.");
  }
  
  if(!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  }

  Serial.println("Wall Following Script:\n");


  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();  // Automatically calibrate the gyroscope
  Serial.println("Done!");

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  #if DEBUG
  Serial.println("Debug mode is ON");
  Serial.print("Ultra Sound Sensors Initialised \n");
  if (myIMU.init()) {
    Serial.println("ICM20948 is connected");
  }
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:

  // Iterate for each IR distance sensor used (in case we add more)
  for (int i=0; i<n; i++) {
    readDistance(i);
    Serial.print("Distance Sensor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(distance[i]);
    Serial.println("cm");
  }
  // Checks both (two) ultrasonic sensors
  readUltrasonic();
  Serial.print("Wall distance: ");
  Serial.print(u_distance[0]);
  Serial.println(" cm");
  // Check if the object is within 10 cm (sensor one - change to front or back)
  if ((u_distance[0] < 15.0) && (u_distance[0] > 1)) {
    Serial.print("Wall detected Ultrasound Sensor 1 (distance < 5 cm):");
    Serial.print(u_distance[0], 2);
    Serial.println(" cm");
    wallMode = TURNING; // Change to turning mode
  }

  if (wallMode == TURNING) {
    IMU(); // Read angle[]
    if (!isTurning) startAngle = angle[2];
    isTurning = true; // Set turning flag
    turnAngle(turnDirection);
  }
  else {
    PID(); // Call PID function
  }

  // PID();
  // Serial.println("PID Called");

  // delay(100); default
  delay(200); // Change for reading Serial data
}

void readDistance(int sensor)
{
      float voltage_temp_average=0;
      
      for(int i=0; i < AVERAGE_OF; i++)
    {
      int sensorValue = analogRead(sensorPin[sensor] );
      delayMicroseconds(1000);      
      voltage_temp_average +=sensorValue * (MCU_VOLTAGE / 1023.0);

    }
     voltage_temp_average /= AVERAGE_OF; // /= operator is equivalent to voltage_temp_average = voltage_temp_average / AVERAGE_OF

  // equation of the fitting curve found using data values
  // << 33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4 >>
  distance[sensor] = 33.9 + -69.5*(voltage_temp_average) + 62.3*pow(voltage_temp_average,2) + -25.4*pow(voltage_temp_average,3) + 3.83*pow(voltage_temp_average,4); 
  if (distance[sensor] > 20) {
    distance[sensor] = -1;
  }
}

void readUltrasonic(void) 
{
  // Clear the trigger pin
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond HIGH pulse
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);

  // Measure the duration of the echo pulse
  u_duration[0] = pulseIn(echoPin1, HIGH,20000);
  u_duration[1] = pulseIn(echoPin2, HIGH,20000); // duration of second ultrasound

  // Calculate the distance in centimeters
  u_distance[0] = u_duration[0] * 0.034 / 2;
  u_distance[1] = u_duration[1] * 0.034 / 2;

  // delay(500);  // Wait before next measurement
}

void PID() {
  // --- Get IR sensor values ---
  float leftFront = distance[0]; // A0
  float leftBack = distance[1]-0.35;  // A1

  // PID gains distance
  const float distanceKp = 30;
  const float distanceKi = 0;
  const float distanceKd = 0;

  // PID gains angle
  const float angleKp = 500;
  const float angleKi = 0;
  const float angleKd = 0;

  static float prevDistanceError = 0;
  static float distanceIntegral = 0;
  static float prevAngleError = 0;
  static float angleIntegral = 0;
  static unsigned long lastTime = 0; // for dt
  unsigned long currentTime = millis(); 

  float dt = (currentTime - lastTime) / 1000.0; // convert to seconds
  lastTime = currentTime;

  if (leftFront > 0 && leftBack > 0) { // valid values
    float targetDistance = 4.2;

    // // Compute errors
    // float distanceError = ((leftFront + leftBack) / 2.0) - targetDistance;
    // float angleError = leftFront - leftBack;

    // Compute errors
    float difference = distance[0] - distance[1]; // front - back
    float meanDistance = (distance[0] + distance[1]) / 2.0;
    angleError = atan2(difference, sensorSeparation); // angle in radians (target angle = 0)
    distanceError = (meanDistance * cos(angleError)) - targetDistance; // perpendicular distance to the wall
    if (dt <= 0) dt = 0.001; // Prevent division by zero

    // Integral terms
    distanceIntegral += distanceError * dt; // provided dt > 0
    angleIntegral += angleError * dt;

    // Derivative terms
    float distanceDerivative = (distanceError - prevDistanceError) / dt;
    float angleDerivative = (angleError - prevAngleError) / dt;

    // PID output
    float totalDistanceError = distanceKp * distanceError + distanceKi * distanceIntegral + distanceKd * distanceDerivative;
    float totalAngleError = angleKp * angleError + angleKi * angleIntegral + angleKd * angleDerivative;

    // Calculate motor speeds
    float baseSpeed = 300; 
    float leftSpeed = baseSpeed - totalDistanceError - totalAngleError;
    float rightSpeed = baseSpeed + totalDistanceError + totalAngleError;

    // float leftSpeed = baseSpeed + totalDistanceError - totalAngleError;

    // Set turning motor speeds
    setGreenLeft(leftSpeed);
    setGreenRight(rightSpeed);
    Serial.print("Green Left Speed : ");
    Serial.println(leftSpeed);

    Serial.print("Green Right Speed : ");
    Serial.println(rightSpeed);

    prevDistanceError = distanceError;
    prevAngleError = angleError;
  } else { // If sensor values are invalid
    float baseSpeed = 300;
    // setLeftMotors(baseSpeed); // **change to back left motor**
    // setRightMotors(baseSpeed); // **change to back right motor**
  }
}

void setMotor(int motor, int speed) {
  speed = constrain(speed, -800, 800);
  if (orientation == LINE_TRACKING) speed = -speed; // Reverse speed for line tracking
  switch (motor) {
    case 1: mc2.setSpeed(1, -speed); break;
    case 2: mc1.setSpeed(1, speed); break;
    case 3: mc2.setSpeed(3, -speed); break;
    case 4: mc1.setSpeed(3, speed); break;
  }
}

void setLeftMotors(int speed) {
  setMotor(2, speed); // Green left (Black is FRONT)
  setMotor(1, speed); // Black left
}

void setRightMotors(int speed) {
  setMotor(4, speed); // Green right
  setMotor(3, speed); // Black right
}

void setGreenLeft(int speed) {
  setMotor(4, speed); // Green left
}

void setBlackLeft(int speed) {
  setMotor(1, speed); // Black left
}

void setGreenRight(int speed) {
  setMotor(2, speed);
}

void setBlackRight(int speed) {
  setMotor(3, speed);
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

void setServoMax() {
  int pos;
  int initPos = myservo.read();

  for (pos = 25; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
}

void setServoMin() {
  int pos;
  int initPos = myservo.read();

  for (pos = initPos; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
}

void setServoFlat() {
  int pos;
  int initPos = myservo.read();

  // Decrement position
  if (initPos > 30) {
    for (pos = initPos; pos <= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
    }
  }

  // Increment position
  else if (initPos < 30) {
    for (pos = initPos; pos >= 30; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
    }
  } 
}

void IMU() {
  float angleX = 0.0;
  float angleY = 0.0;
  float angleZ = 0.0;
  static unsigned long lastTime = millis();      // Initialize time
  
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

  // Store in global array
  angle[0] = angleX;
  angle[1] = angleY;
  angle[2] = angleZ;

  #if DEBUG
    // Serial.print("Angular velocity (°/s): X=");
    // Serial.print(gyr.x);
    // Serial.print(" Y=");
    // Serial.print(gyr.y);
    // Serial.print(" Z=");
    // Serial.println(gyr.z);

    // Output current angle information

    Serial.print("Accumulated angle (°): X=");
    Serial.print(angleX);
    Serial.print(" Y=");
    Serial.print(angleY);
    Serial.print(" Z=");
    Serial.println(angleZ);
  #endif

  delay(20); // Approx. 50Hz sampling
}

void turnRight() { // Needs editing and testing deprecated
  // Turn right by setting the left motor to a negative speed and the right motor to a positive speed 
  setGreenLeft(800);
  setGreenRight(-800);
  Serial.println("Turning Right...");
  delay(2000);
}

void turnAngle(TurnDirection turnDirection) {
  float targetAngle = 0;
  int leftSpeed = 0;
  int rightSpeed = 0;

  // Calculate target angle based on the current angle and the turn direction
  switch (turnDirection) {
    case CLOCKWISE:
      targetAngle = 90.0;
      leftSpeed = 800;
      rightSpeed = -800;
      Serial.println("Turning Clockwise...");
      break;
    case ANTICLOCKWISE:
      targetAngle = -90.0;
      leftSpeed = -800;
      rightSpeed = 800;
      Serial.println("Turning Anticlockwise...");
      break;
    case FLIP:
      targetAngle = 180.0;
      leftSpeed = 800;
      rightSpeed = -800;
      Serial.println("Flipping...");
      break;
    default:
      return;
  }
  float currentAngle = angle[2];
  float angleDifference = currentAngle - startAngle; // Calculate the difference from the starting angle

  // Handle wrap-around for angles
  if (angleDifference > 180.0) {
    angleDifference -= 360.0;
  } else if (angleDifference < -180.0) {
    angleDifference += 360.0;
  }

  bool finishedTurning = false;
  if (turnDirection == FLIP) {
    finishedTurning = fabs(angleDifference) >= fabs(targetAngle);
  } else { // logic for 90degree turns
    finishedTurning = (turnDirection == CLOCKWISE && angleDifference >= targetAngle) ||
    (turnDirection == ANTICLOCKWISE && angleDifference <= targetAngle);
  }

  if (!finishedTurning) {
    setGreenLeft(leftSpeed);
    setGreenRight(rightSpeed);
    
    #if DEBUG
      Serial.print("Turning... Current angle: ");
      Serial.print(currentAngle);
      Serial.print(" Target angle: ");
      Serial.println(targetAngle);
    #endif
  } else {
    setGreenLeft(0);
    setGreenRight(0);
    isTurning = false; // Reset turning flag
    wallMode = WALL_FOLLOW; // Change back to wall following mode

    Serial.println("Turn completed. Returning to wall following mode.");
  }

}