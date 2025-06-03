#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Motoron.h>

const char* ssid = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";
int i = 1;
int j = 0;
// const char* ssid = "hajimi666";
// const char* password = "password";


WiFiUDP udp;
unsigned int localPort = 55500;
char packetBuffer[64];
bool motorsStopped = false;

MotoronI2C m1(0x10);
MotoronI2C m2(0x50);

const int buttonPin = 2;
bool lastButtonState = HIGH;
bool buttonPressed = false;

void stopAllMotors() {
  m1.setSpeed(1, 0);
  m2.setSpeed(1, 0);
  m1.setSpeed(3, 0);
  m2.setSpeed(3, 0);
  motorsStopped = true;
  Serial.println("Motors stopped.");
  digitalWrite(LED_BUILTIN, HIGH);
}

void startAllMotors(int i) {
  m1.setSpeed(1, i*200);
  m2.setSpeed(1, i*200);
  m1.setSpeed(3, i*-200);
  m2.setSpeed(3, i*200);
  motorsStopped = false;
  Serial.println("Motors started.");
  digitalWrite(LED_BUILTIN, LOW);

}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  Wire.begin();
  m1.setBus(&Wire);
  m2.setBus(&Wire);

  m1.reinitialize();
  m1.disableCrc();
  m1.disableCommandTimeout();
  m1.clearResetFlag();
  m1.setMaxAcceleration(1, 1000);
  m1.setMaxAcceleration(3, 1000);

  m2.reinitialize();
  m2.disableCrc();
  m2.disableCommandTimeout();
  m2.clearResetFlag();
  m2.setMaxAcceleration(1, 1000);
  m2.setMaxAcceleration(3, 1000);

  startAllMotors(i);
}

void loop() {
  // --- Handle UDP message ---
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = '\0';

    Serial.print("Received UDP message: ");
    Serial.println(packetBuffer);

    if (strcmp(packetBuffer, "Stop") == 0) {
      stopAllMotors();
    }
  }

  // --- Handle button press ---
  bool currentState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentState == LOW) {
    buttonPressed = true;
  }
  lastButtonState = currentState;

  if (buttonPressed) {
    buttonPressed = false;

    if (motorsStopped) {
      startAllMotors(i);
      if(j%100 == 0){
        i = i*-1;
        
      }
      j++;
    } else {
      stopAllMotors();
    }
  }

  delay(10);
}



// code with simple action to move forward:


// #include <WiFiS3.h>
// #include <WiFiUdp.h>
// #include <Wire.h>
// #include <Motoron.h>

// const char* ssid = "PhaseSpaceNetwork_2.4G";
// const char* password = "PhaseSpaceNetwork_2.4G";

// // const char* ssid = "hajimi666";
// // const char* password = "password";


// WiFiUDP udp;
// unsigned int localPort = 55500;
// char packetBuffer[64];
// bool motorsStopped = false;

// MotoronI2C m1(0x10);
// MotoronI2C m2(0x50);

// const int buttonPin = 2;
// bool lastButtonState = HIGH;
// bool buttonPressed = false;

// void stopAllMotors() {
//   m1.setSpeed(1, 0);
//   m2.setSpeed(1, 0);
//   m1.setSpeed(3, 0);
//   m2.setSpeed(3, 0);
//   motorsStopped = true;
//   Serial.println("Motors stopped.");
//   digitalWrite(LED_BUILTIN, HIGH);
// }

// void startAllMotors() {
//   m1.setSpeed(1, 200);
//   m2.setSpeed(1, 200);
//   m1.setSpeed(3, 200);
//   m2.setSpeed(3, 200);
//   motorsStopped = false;
//   Serial.println("Motors started.");
//   digitalWrite(LED_BUILTIN, LOW);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(buttonPin, INPUT_PULLUP);

//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }

//   Serial.println("\nWi-Fi connected.");
//   Serial.print("Local IP address: ");
//   Serial.println(WiFi.localIP());

//   udp.begin(localPort);

//   Wire.begin();
//   m1.setBus(&Wire);
//   m2.setBus(&Wire);

//   m1.reinitialize();
//   m1.disableCrc();
//   m1.disableCommandTimeout();
//   m1.clearResetFlag();
//   m1.setMaxAcceleration(1, 1000);
//   m1.setMaxAcceleration(3, 1000);

//   m2.reinitialize();
//   m2.disableCrc();
//   m2.disableCommandTimeout();
//   m2.clearResetFlag();
//   m2.setMaxAcceleration(1, 1000);
//   m2.setMaxAcceleration(3, 1000);

//   startAllMotors();
// }

// void loop() {
//   // --- Handle UDP message ---
//   int packetSize = udp.parsePacket();
//   if (packetSize) {
//     int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
//     if (len > 0) packetBuffer[len] = '\0';

//     Serial.print("Received UDP message: ");
//     Serial.println(packetBuffer);

//     if (strcmp(packetBuffer, "Stop") == 0) {
//       stopAllMotors();
//     }
//   }

//   // --- Handle button press ---
//   bool currentState = digitalRead(buttonPin);

//   if (lastButtonState == HIGH && currentState == LOW) {
//     buttonPressed = true;
//   }
//   lastButtonState = currentState;

//   if (buttonPressed) {
//     buttonPressed = false;

//     if (motorsStopped) {
//       startAllMotors();
//     } else {
//       stopAllMotors();
//     }
//   }

//   delay(10);
// }