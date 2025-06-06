// Ultrasonic Sensor HC-SR04 Distance Measurement

const int trigPin = 8;
const int echoPin = 9;

long duration;
float distance;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Sensor initialized.");
}

void loop() {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond HIGH pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  // Check if the object is within 10 cm
  if (distance < 10.0) {
    Serial.print("Wall detected (distance < 10 cm):");
    Serial.print(distance, 2);
    Serial.println(" cm");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance, 2);
    Serial.println(" cm");
  }

  delay(500);  // Wait before next measurement
}