int pressurePin = 1;  // ESP32 ADC input pin
int pressureValue = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Set ADC resolution (0-4095)
}

void loop() {
  pressureValue = analogRead(pressurePin);  // Read the amplified voltage
  Serial.println(pressureValue);  // Print the raw ADC value
  delay(100);  // Delay to avoid flooding serial output
}