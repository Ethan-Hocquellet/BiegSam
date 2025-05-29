void setup() {

  Serial.begin(9600);
  // Wait for GIGA
  while (!Serial);

  int State = digitalRead(PC_13);
  Serial.print("State of PC13: ");
  Serial.print(State);
  Serial.println("Hello World \n");
}

void loop() {
  // nothing :)
  delay(50);
}
