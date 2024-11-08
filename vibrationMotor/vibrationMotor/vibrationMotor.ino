const int VibPin = 21; // GPIO pin connected to the Vibration Module

void setup() {
  pinMode(VibPin, OUTPUT); // Set the vibration pin as output
}

void loop() {
  digitalWrite(VibPin, HIGH);
  delay(300);
  digitalWrite(VibPin, LOW);
  delay(2000);
}
