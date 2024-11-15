const uint8_t pressureSheetPins[4] = {1, 2, 3, 4};
uint16_t pressureSheetValues[4] = {0, 0, 0, 0};
uint16_t pressureSheetThresholds[4] = {0, 0, 0, 0};

// TODO: stuur m/data met data 1 - 4

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
  
  for (int i = 0; i < 4; i++) {
    pressureSheetThresholds[i] = analogRead(pressureSheetPins[i]) / 2;
  }
}

void loop() {
  for (int i = 0; i < 4; i++) {
    pressureSheetValues[i] = analogRead(pressureSheetPins[i]);
    if (pressureSheetValues[i] < pressureSheetThresholds[i]) {
      Serial.println(pressureSheetPins[i]);
    }
  }
}
