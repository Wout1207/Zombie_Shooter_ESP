const int vib_pin = 21; // GPIO pin connected to the Vibration Module

void setup() {
  pinMode(vib_pin, OUTPUT); // Set the vibration pin as output
}

void loop() {
  digitalWrite(vib_pin, HIGH);
  delay(300);
  digitalWrite(vib_pin, LOW);
  delay(2000);
}
