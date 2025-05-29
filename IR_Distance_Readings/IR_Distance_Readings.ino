/*
 * 
 * 
This is Arduino code to measure the distnace using Sharp Infrared Distance
sensor GP2Y0A51SK0F to measure from 2 to 14cm

See datasheet 
https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a51sk_e.pdf
 * 
 * Watch Video instrution for this code:  https://youtu.be/rdNCIyL6OA8
 * 

 * Written by Ahmad Shamshiri on July 09 2020 at in Ajax, Ontario, Canada
 * in Ajax, Ontario, Canada. www.robojax.com
 * 

 * Get this code and other Arduino codes from Robojax.com
Learn Arduino step by step in structured course with all material, wiring diagram and library
all in once place. Purchase My course on Udemy.com http://robojax.com/L/?id=62

If you found this tutorial helpful, please support me so I can continue creating 
content like this. You can support me on Patreon http://robojax.com/L/?id=63

or make donation using PayPal http://robojax.com/L/?id=64
Related videos:
Introduction to MAX6675 K-Type: https://youtu.be/VGqONmUinqA
Using MAX6675 K-Type thermocouple whit LED display: https://youtu.be/cD5oOu4N_AE
Using MAX6675 K-Type thermocouplewith LCD1602-I2C: https://youtu.be/BlhpktgPdKs
Using 2 or more MAX6675 K-Type thermocouple: https://youtu.be/c90NszbNG8c
Using MAX6675 K-Type thermocouple as Heater or Coller controller: [this video]
 *  * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


*/

//33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4


const int sensorPin[] = {A0};
float distance[1];//watch video for details https://youtu.be/rdNCIyL6OA8
const int AVERAGE_OF =50;
const float MCU_VOLTAGE = 5.0;

void setup()
{
 //Robojax.com code for sharp IR sensor 
  Serial.begin(115200); 
  // Serial.println("Robojax Sharp GP2Y0A51SK0F demo");
 //Robojax.com code for sharp IR sensor 
}//setup ends here



void loop(){
  //Robojax.com code for sharp IR sensor 
  int rawValue = analogRead(sensorPin);
  float voltage = rawValue * (5 / 4095.0); // At 5V

  // print out the value you read:
  Serial.print("Raw ADC: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | ");

  readDistance(0);//read sensor 1
  // readDistance(1);//read sensor 2
  if(distance[0] > 20.0)
  {
    Serial.println("Distance: Out of Range!");
  } else{ 
    Serial.print("Distance: ");// 
    Serial.print(distance[0]);
    Serial.println("cm");
  }
  delay(300);
 
}//loop end


/*
 *  readDistance()
 * @brief reads the the distance from sharp sensor
 * and updates the apropriate sensor's value
 * @param "sensor" is integer

 * @return none
 * Written by Ahmad Shamshiri for robojax.com
 * on July 09, 2020 in Ajax, Ontario, Canada
 */
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
     voltage_temp_average /= AVERAGE_OF;

  // equation of the fitting curve
  ////33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
  distance[sensor] = 33.9 + -69.5*(voltage_temp_average) + 62.3*pow(voltage_temp_average,2) + -25.4*pow(voltage_temp_average,3) + 3.83*pow(voltage_temp_average,4);
  
     //Robojax.com code for sharp IR sensor 
}//readDistance
 