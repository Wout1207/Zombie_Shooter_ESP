#define CONVERSIONS_PER_PIN 10

bool grenadeThrown = false;
bool grenadeDetected[] = {false, false, false, false, false, false};
uint8_t photoTransistorPins[] = {1,2,3,4,5,6};
uint16_t photoTransistorReferences[] = {0,0,0,0,0,0};
uint16_t photoTransistorValues[] = {0,0,0,0,0,0};
uint16_t photoTransistorHighDeviations[] = {0,0,0,0,0,0};
uint16_t photoTransistorLowDeviations[] = {0,0,0,0,0,0};

uint8_t counter = 0;

// Calculate how many pins are declared in the array - needed as input for the setup function of ADC Continuous
uint8_t adc_pins_count = 6;

// Flag which will be set in ISR when conversion is done
volatile bool adc_coversion_done = false;

// Result structure for ADC Continuous reading
adc_continuous_data_t *result = NULL;

// ISR Function that will be triggered when ADC conversion is done
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

// ================================================================
// ===               SETUP FUNCTION                             ===
// ================================================================

void setup() {
  Serial.begin(115200);
  // Optional for ESP32: Set the resolution to 9-12 bits (default is 12 bits)
  analogContinuousSetWidth(12);
  // Optional: Set different attenaution (default is ADC_11db)
  analogContinuousSetAtten(ADC_11db);
  // Setup ADC Continuous with following input:
  // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
  analogContinuous(photoTransistorPins, adc_pins_count, CONVERSIONS_PER_PIN, 20000, &adcComplete);
  // Start ADC Continuous conversions
  analogContinuousStart();
  counter = 9;
  // photoTransistorsSetup();

}

// ================================================================
// ===               LOOP FUNCTION                              ===
// ================================================================

void loop() {
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    if (counter < 9) {
      photoTransistorsLoop();
      // counter++;
    }
    else {
      photoTransistorsSetup();
      counter = 0;
    }
  }

}

// ================================================================
// ===               PHOTO TRANSISTORS SETUP                    ===
// ================================================================


void photoTransistorsSetup() {
  if (analogContinuousRead(&result, 0)) {
    analogContinuousStop();
    for (int i = 0; i < adc_pins_count; i++) {
      photoTransistorReferences[i] = result[i].avg_read_raw;
      photoTransistorHighDeviations[i] = photoTransistorReferences[i] * 0.2;
      photoTransistorLowDeviations[i] = photoTransistorReferences[i] * 0.1;
      Serial.println (i);
      Serial.print("Reference :");
      Serial.println(photoTransistorReferences[i]);
      Serial.print("High deviation :");
      Serial.println(photoTransistorHighDeviations[i]);
      Serial.print("Low deviation :");
      Serial.println(photoTransistorLowDeviations[i]);
    }
    analogContinuousStart();
  } else {
    Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
  }
}

// ================================================================
// ===               PHOTO TRANSISTORS LOOP                     ===
// ================================================================


void photoTransistorsLoop() {
  // Read data from ADC
  if (analogContinuousRead(&result, 0)) {
    analogContinuousStop();
    bool allGrenadesCleared = true;
    for (int i = 0; i < adc_pins_count; i++) {
      photoTransistorValues[i] = result[i].avg_read_raw;
      if (!grenadeDetected[i]) {
        if (photoTransistorValues[i] > photoTransistorReferences[i] + photoTransistorHighDeviations[i] ||
            photoTransistorValues[i] < photoTransistorReferences[i] - photoTransistorHighDeviations[i]) {
          // Serial.print("Pin: ");
          // Serial.print(photoTransistorPins[i]);
          // Serial.print(" Value: ");
          // Serial.print(photoTransistorValues[i]);
          // Serial.println(" Grenade detected");
          grenadeDetected[i] = true;
        }
      }
      else {
        if (photoTransistorValues[i] < photoTransistorReferences[i] + photoTransistorLowDeviations[i] &&
            photoTransistorValues[i] > photoTransistorReferences[i] - photoTransistorLowDeviations[i]) {
          // Serial.print("Pin: ");
          // Serial.print(photoTransistorPins[i]);
          // Serial.print(" Value: ");
          // Serial.print(photoTransistorValues[i]);
          // Serial.println(" No Grenade detected");
          grenadeDetected[i] = false;
        }
      }

      if (grenadeDetected[i]) {
        allGrenadesCleared = false;
      }
    }

    // If any grenade is detected, set grenadeThrown to true and send the message
    if (!grenadeThrown && !allGrenadesCleared) {
      grenadeThrown = true;
      Serial.println("Grenade thrown! Message sent.");
    }
    // Reset grenadeThrown to false if all grenades are cleared
    if (grenadeThrown && allGrenadesCleared) {
      grenadeThrown = false;
      Serial.println("Grenades cleared.");
    }
    
    analogContinuousStart();
  } else {
    Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
  }
}


