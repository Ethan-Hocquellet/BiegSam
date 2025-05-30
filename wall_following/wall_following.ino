const int n = 1; // Number of IR Distance
const int u_n = 2; // Number of Ultrasonic
const int sensorPin[] = {A0}; // n must match array size of sensorPin[]}
float distance[n];
float u_distance[u_n];
long duration[u_n]; // duration for two ultrasonic sensors
float u_duration[u_n];

const int trigPin1 = 8;
const int echoPin1 = 9;
const int trigPin2 = 10; // change for correct pins
const int echoPin2 = 11;


#define AVERAGE_OF 50
#define MCU_VOLTAGE 5
// const int AVERAGE_OF = 50;
// const float MCU_VOLTAGE = 5.0;

void readDistance(int sensor);
// @readDistance for GP2Y0A51SK0F
void readUltrasonic(void);
// @readUltrasonic for Ultrasonic


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);


  Serial.print("Ultra Sound Sensors Initialised \n");
  
  Serial.println("Wall Following Script:\n");
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
  // Check if the object is within 10 cm (sensor one - change to front or back)
  if (u_distance[0] < 10.0) {
    Serial.print("Wall detected Ultrasound Sensor 1 (distance < 10 cm):");
    Serial.print(u_distance[0], 2);
    Serial.println(" cm");
  } else {
    Serial.print("Distance: ");
    Serial.print(u_distance[0], 2);
    Serial.println(" cm");
  }

  // Change to front or back
  if (distance[1] < 10.0) {
    Serial.print("Wall detected Ultrasound Sensor 2 (distance < 10 cm):");
    Serial.print(u_distance[1], 2);
    Serial.println(" cm");
  } else {
    Serial.print("Distance: ");
    Serial.print(u_distance[1], 2);
    Serial.println(" cm");
  }

  delay(500); // Necessary for ultrasonic?
}

void readDistance(int sensor)
{
      float voltage_temp_average=0;
      
      for(int i=0; i < AVERAGE_OF; i++)
    {
      int sensorValue = analogRead(sensorPin[sensor] );
      delay(1);      
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
  u_duration[0] = pulseIn(echoPin1, HIGH);
  u_duration[1] = pulseIn(echoPin2, HIGH); // duration of second ultrasound

  // Calculate the distance in centimeters
  u_distance[0] = u_duration[0] * 0.034 / 2;
  u_distance[1] = u_duration[1] * 0.034 / 2;

  
  // delay(500);  // Wait before next measurement
}
