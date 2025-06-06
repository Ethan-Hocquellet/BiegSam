#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <math.h>
// #include <WifiS3.h> // Works with Giga
#include <WiFiUdp.h>

#define AVERAGE_OF 50
#define MCU_VOLTAGE 5
#define ICM20948_ADDR 0x68 // ICM20948 address;
#define DEBUG 1            // Set to 1 to enable debug messages
#define MAX_SERVO 140
#define MIN_SERVO 10
#define FLAT_SERVO 25
#define OBSTACLE_SERVO 90

Servo myservo;
MotoronI2C mc1(0x10);
MotoronI2C mc2(0x50);
// WiFiUDP udp;

const int n = 2;                  // Number of IR Distance
const int u_n = 2;                // Number of Ultrasonic
const int sensorPin[] = {A0, A1}; // n must match array size of sensorPin[]}
float distance[n];
float u_distance[u_n];
long duration[u_n]; // duration for two ultrasonic sensors
float u_duration[u_n];
float angleError, distanceError;
float angle[3]; // angle[0] = x, angle[1] = y, angle[2] = z

// WiFi + Kill Switch
// const char* ssid = "PhaseSpaceNetwork_2.4G"; // WiFi SSID
// const char* password = "8igMacNet"; // WiFi Password
// unsigned int localPort = 55500; // Local port for UDP
// char packetBuffer[64]; // Buffer for incoming UDP packets
// bool motorsStopped = false; // Flag to check if motors are stopped
// const int buttonPin = 2;
// bool lastButtonState = HIGH; // Last state of the button
// bool buttonPressed = false; // Flag to check if button is pressed

// Ultrasonic 1
const int trigPin1 = 0;
const int echoPin1 = 1;
// Ultrasonic 2
const int trigPin2 = 11; // change for correct pins
const int echoPin2 = 12;

// turning
float startAngle = 0.0; // Starting angle for turning function
bool isTurning = false;
bool turnCompleted = false; // Flag to check if turn is completed
bool onObstacle = true;

const float sensorSeparation = 23; // distance between the two IR distance sensors

// enums
enum ServoMode
{
    SHORT_MODE,
    LONG_MODE
};
ServoMode servoMode = SHORT_MODE; // Default mode

// define wall following and turning modes
enum WallMode
{
    WALL_FOLLOW,
    TURNING,
    ON_OBSTACLE
};
WallMode wallMode = ON_OBSTACLE;
;

enum SpinDirection
{
    CLOCKWISE,
    ANTICLOCKWISE,
    FLIP,
    STRAIGHT
};
SpinDirection spinDirection = CLOCKWISE; // Default turn direction

enum Orientation
{
    LINE_TRACKING,
    WALL_TRACKING
};                                       // LINE_TRACKING = black first
Orientation orientation = LINE_TRACKING; // Default orientation

enum Obstacle
{
    GIANT_CAUSEWAY,
    STAIRCASE,
    DRAGON_LAVA_PIT,
    TREADMILL,
    ZIPLINE,
    RAMP,
    LUNAR_SURFACE,
    NONE
};

int obstacleCounter = 0;
Obstacle obstacleOrder[] = {
    RAMP,
    STAIRCASE,
    GIANT_CAUSEWAY,
    TREADMILL,
    LUNAR_SURFACE,
};

Obstacle obstacle = obstacleOrder[obstacleCounter]; // Default obstacle
unsigned long wallFollowTime = 0;                   // Time for wall following

// Func prototypes because Sophia told us to :)
void checkWiFi(void);
// @checkWiFi for checking WiFi connection and receiving UDP messages
void checkButton(void);
// @checkButton for checking button state and toggling motors
void stopAllMotors(void);
// @stopAllMotors for stopping all motors
void startAllMotors(int speed);
// @startAllMotors for starting all motors with a specified speed
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
void MotoronSetup(void);
// @MotoronSetup for initialising the Motoron I2C bus
void setServoAngle(int angle);
// @setServoAngle for setting the servo to a specified angle
void IMU(void); // Removed
// @IMU for reading the IMU data and calculating angles
void turnAngle(SpinDirection spinDirection);
// @turnAngle for turning the robot by a specified angle
void doObstacle(Obstacle obstacle);
// @doObstacle for handling obstacles based on the current obstacle state

void setup()
{
    Serial.begin(115200);

    obstacleCounter = 0;
    wallMode = ON_OBSTACLE; // Start in obstacle mode
    // pinMode(LED_BUILTIN, OUTPUT); // Visual indicator
    // pinMode(buttonPin, INPUT_PULLUP);

    // WiFi setup
    // WiFi.begin(ssid, password); // Connect to WiFi

    // while (WiFi.status() != WL_CONNECTED) {
    // delay(500);
    // Serial.print(".");
    // }

    // Serial.println("\nWi-Fi connected.");
    // Serial.print("Local IP address: ");
    // Serial.println(WiFi.localIP());

    // udp.begin(localPort); // Start UDP on local port

    MotoronSetup(); // Initialise Motors
    myservo.attach(52);

    // Ultrasonic sensor setup
    pinMode(trigPin1, OUTPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(echoPin2, INPUT);

    if (servoMode == SHORT_MODE)
    {
        setServoAngle(MAX_SERVO); // Set servo to maximum position
        Serial.println("Servo set to maximum position.");
    }
    else if (servoMode == LONG_MODE)
    {
        setServoAngle(FLAT_SERVO); // Set servo to minimum position
        Serial.println("Servo set to flat position.");
    }

    // setGreenLeft(300);
    // setGreenRight(300);
    // setBlackLeft(300);
    // setBlackRight(300);
}

void loop()
{

    // Check for incoming UDP messages
    // checkWiFi();
    // Check button state
    // checkButton();

    // Iterate for each IR distance sensor used (in case we add more)
    for (int i = 0; i < n; i++)
    {
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

    // Obstacle handling state machine
    if (wallMode == ON_OBSTACLE)
    {
        Serial.print("Handling obstacle: ");
        Serial.println(obstacle);
        doObstacle(obstacleOrder[obstacleCounter]);

        switch (wallMode)
        {
        case ON_OBSTACLE:
            Serial.print("Handling obstacle: ");
            Serial.println(obstacleOrder[obstacleCounter]);
            doObstacle(obstacleOrder[obstacleCounter]);
            if (u_distance[0] < 5.0 && u_distance[0] > 0)
            {
                Serial.println("Wall detected after obstacle.");
                stopAllMotors();
                delay(200);
                wallMode = TURNING;
            }
            break;

        case TURNING:
            Serial.println("Turning right...");
            turnRight();
            stopAllMotors();
            delay(200);
            wallFollowTime = millis();
            wallMode = WALL_FOLLOW;
            break;

        case WALL_FOLLOW:
            PID();
            if (millis() - wallFollowTime > 1000)
            { // 1 second wall follow
                stopAllMotors();
                obstacleCounter++;
                if (obstacleCounter < (sizeof(obstacleOrder) / sizeof(obstacleOrder[0])))
                {
                    wallMode = ON_OBSTACLE;
                }
                else
                {
                    Serial.println("All obstacles completed.");
                    while (1)
                        ; // Stop execution
                }
            }
            break;
        }
    }

    // Wall following and turning
    if ((u_distance[0] < 10.0) && (u_distance[0] > 1))
    {
        // Wall detected, handle turning if needed
        if (wallMode == TURNING)
        {
            turnRight();
        }
        else
        {
            PID();
        }
    }
    else
    {
        // Wall lost, go to next obstacle
        obstacleCounter++;
        if (obstacleCounter < (sizeof(obstacleOrder) / sizeof(obstacleOrder[0])))
        {
            obstacle = obstacleOrder[obstacleCounter];
            onObstacle = true;
            Serial.println("Wall lost, switching to next obstacle.");
        }
    }
    delay(200); // Change for reading Serial data
}

// void checkWiFi() {
// int packetSize = udp.parsePacket();
// if (packetSize) {
// int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
// if (len > 0) packetBuffer[len] = '\0';

// Serial.print("Received UDP message: ");
// Serial.println(packetBuffer);

// if (strcmp(packetBuffer, "Stop") == 0) {
// stopAllMotors();
// motorsStopped = true;
// Serial.println("Motors stopped.");
// } else if (strcmp(packetBuffer, "Start") == 0) {
// startAllMotors(200);
// motorsStopped = false;
// Serial.println("Motors started.");
// }
// }
// }

// void checkButton() {
// bool currentState = digitalRead(buttonPin); // Read the current state of the button

// if (lastButtonState == HIGH && currentState == LOW) {
// buttonPressed = true;
// }
// lastButtonState = currentState;

// if (buttonPressed) {
// buttonPressed = false;

// if (motorsStopped) {
// startAllMotors(200);
// } else {
// stopAllMotors();
// }
// }
// }

void readDistance(int sensor)
{
    float voltage_temp_average = 0;
    for (int i = 0; i < AVERAGE_OF; i++)
    {
        int sensorValue = analogRead(sensorPin[sensor]);
        delayMicroseconds(1000);
        voltage_temp_average += sensorValue * (MCU_VOLTAGE / 1023.0);
    }
    voltage_temp_average /= AVERAGE_OF; // /= operator is equivalent to voltage_temp_average = voltage_temp_average / AVERAGE_OF

    // equation of the fitting curve found using data values
    // << 33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4 >>
    distance[sensor] = 33.9 + -69.5 * (voltage_temp_average) + 62.3 * pow(voltage_temp_average, 2) + -25.4 * pow(voltage_temp_average, 3) + 3.83 * pow(voltage_temp_average, 4);
    if (distance[sensor] > 20)
    {
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
    u_duration[0] = pulseIn(echoPin1, HIGH, 20000);
    u_duration[1] = pulseIn(echoPin2, HIGH, 20000); // duration of second ultrasound

    // Calculate the distance in centimeters
    u_distance[0] = u_duration[0] * 0.034 / 2;
    u_distance[1] = u_duration[1] * 0.034 / 2;

    // delay(500); // Wait before next measurement
}

void PID()
{
    // --- Get IR sensor values ---
    float leftFront = distance[0];       // A0
    float leftBack = distance[1] - 0.35; // A1

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

    if (leftFront > 0 && leftBack > 0)
    { // valid values
        float targetDistance = 4.2;

        // // Compute errors
        // float distanceError = ((leftFront + leftBack) / 2.0) - targetDistance;
        // float angleError = leftFront - leftBack;

        // Compute errors
        float difference = distance[0] - distance[1]; // front - back
        float meanDistance = (distance[0] + distance[1]) / 2.0;
        angleError = atan2(difference, sensorSeparation);                  // angle in radians (target angle = 0)
        distanceError = (meanDistance * cos(angleError)) - targetDistance; // perpendicular distance to the wall
        if (dt <= 0)
            dt = 0.001; // Prevent division by zero

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
    }
    else
    { // If sensor values are invalid
        float baseSpeed = 300;
        // setLeftMotors(baseSpeed); // **change to back left motor**
        // setRightMotors(baseSpeed); // **change to back right motor**
    }
}

void setMotor(int motor, int speed)
{
    speed = constrain(speed, -800, 800);
    if (orientation == LINE_TRACKING)
        speed = -speed; // Reverse speed for line tracking
    switch (motor)
    {
    case 1:
        mc2.setSpeed(1, -speed);
        break;
    case 2:
        mc1.setSpeed(1, speed);
        break;
    case 3:
        mc2.setSpeed(3, -speed);
        break;
    case 4:
        mc1.setSpeed(3, -speed);
        break;
    }
}

void stopAllMotors()
{
    // Stop all motors by setting speed to 0
    setMotor(1, 0);
    setMotor(2, 0);
    setMotor(3, 0);
    setMotor(4, 0);
#if DEBUG
    Serial.println("All motors stopped.");
#endif
}

void startAllMotors(int speed)
{
    // Start all motors with the specified speed
    setGreenLeft(speed);
    setGreenRight(speed);
    setBlackLeft(speed);
    setBlackRight(speed);
#if DEBUG
    Serial.println("All motors started.");
#endif
}

void setLeftMotors(int speed)
{
    setMotor(2, speed); // Green left (Black is FRONT)
    setMotor(1, speed); // Black left
}

void setRightMotors(int speed)
{
    setMotor(4, speed); // Green right
    setMotor(3, speed); // Black right
}

void setGreenLeft(int speed)
{
    setMotor(4, speed); // Green left
}

void setBlackLeft(int speed)
{
    setMotor(1, speed); // Black left
}

void setGreenRight(int speed)
{
    setMotor(2, speed);
}

void setBlackRight(int speed)
{
    setMotor(3, speed);
}

void MotoronSetup()
{
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

void setServoAngle(int angle)
{
    for (int pos = FLAT_SERVO; pos <= angle; pos += 1)
    {
        myservo.write(pos); // tell servo to go to position in variable 'pos'
        delay(15);
    }
}

void turnRight()
{ // Needs editing and testing deprecated
    // Turn right by setting the left motor to a negative speed and the right motor to a positive speed
    setGreenLeft(600);
    setGreenRight(-600);
    Serial.println("Turning Right...");
    delay(100);
}

void doObstacle(Obstacle obstacle)
{
    delay(200);
    switch (obstacle)
    {
    case GIANT_CAUSEWAY:
        myservo.write(OBSTACLE_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling GIANT_CAUSEWAY obstacle.");
        break;
    case STAIRCASE:
        myservo.write(MIN_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling STAIRCASE obstacle.");
        break;
    case DRAGON_LAVA_PIT:
        myservo.write(FLAT_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling DRAGON_LAVA_PIT obstacle.");
        break;
    case TREADMILL:
        myservo.write(FLAT_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling TREADMILL obstacle.");
        break;
    case ZIPLINE:
        myservo.write(FLAT_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling ZIPLINE obstacle.");
        break;
    case RAMP:
        myservo.write(OBSTACLE_SERVO); // Set servo to obstacle position
        setLeftMotors(600);
        setRightMotors(600);
        Serial.println("Handling RAMP obstacle.");
        break;
    case LUNAR_SURFACE:
        myservo.write(OBSTACLE_SERVO); // Set servo to obstacle position
        setLeftMotors(400);
        setRightMotors(400);
        Serial.println("Handling LUNAR_SURFACE obstacle.");
        break;
    default:
    }
}