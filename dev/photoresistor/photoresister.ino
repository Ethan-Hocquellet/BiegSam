int ledPin = 2;

void setup() {
    Serial.begin(9600);
    pinMode(ledPin,  OUTPUT);
}

void loop() {
    int value = analogRead(A0);

    Serial.println("Analog  Value: ");
    Serial.println(value);
    
    if (value > 800) {
        digitalWrite(ledPin, LOW);
    } else {
        digitalWrite(ledPin,  HIGH);
    }
    
    delay(250);
}
