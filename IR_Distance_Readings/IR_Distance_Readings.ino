/*
  REFERENCE https://www.youtube.com/watch?v=rdNCIyL6OA8
  Robojax.com
*/

int n = 1; // number of IR distance sensors
const int sensorPin[] = {A0}; // Must be the same array size as n
float distance[n]; // For recording distance of each sensor
const int AVERAGE_OF = 50; 
const float MCU_VOLTAGE = 5.0; // Assuming 5V

void setup()
{
  Serial.begin(115200); 
  // Serial.println("IR Distance sensor working...");
}


void loop(){ 
  int rawValue = analogRead(sensorPin);
  float voltage = rawValue * (5 / 4095.0); // At 5V

  // print out the value you read:
  Serial.print("Raw ADC: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | ");

  readDistance(0);// read value of first sensor
  if(distance[0] > 20.0) // not accurate over 20cm
  {
    Serial.println("Distance: Out of Range!");
  } else{ 
    Serial.print("Distance: ");// 
    Serial.print(distance[0]);
    Serial.println("cm");
  }
  delay(300);
 
}

void readDistance(int sensor)
{
  //Robojax.com code for sharp IR sensor 
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
}//readDistance
 