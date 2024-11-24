bool grenadeDetected = false;
const uint8_t photoTransistorPins[6] = {35,36,37,38,39,40};
uint16_t photoTransistorReferences[6] = {0,0,0,0,0,0};
uint16_t photoTransistorValues[6] = {0,0,0,0,0,0};
uint16_t photoTransistorHighDeviations[6] = {0,0,0,0,0,0};
uint16_t photoTransistorLowDeviations[6] = {0,0,0,0,0,0};


// ================================================================
// ===               SETUP FUNCTION                             ===
// ================================================================

void setup() {

  Serial.begin(115200);

  photoTransistorsSetup();

}

// ================================================================
// ===               LOOP FUNCTION                              ===
// ================================================================

void loop() {

  photoTransistorsLoop();

}

// ================================================================
// ===               PHOTO TRANSISTORS SETUP                    ===
// ================================================================

void photoTransistorsSetup() {

    for (int i=0; i<6; i++) {
    pinMode(photoTransistorPins[i], INPUT);
    photoTransistorReferences[i] = analogRead(photoTransistorPins[i]);
    photoTransistorHighDeviations[i] = photoTransistorReferences[i] * 0.2;
    photoTransistorLowDeviations[i] = photoTransistorReferences[i] * 0.1;

    // TODO: zodra het werkt onderstaande lijnen verwijderen
    Serial.println (i);
    Serial.print("Reference :");
    Serial.println(photoTransistorReferences[i]);
    Serial.print("High deviation :");
    Serial.println(photoTransistorHighDeviations[i]);
    Serial.print("Low deviation :");
    Serial.println(photoTransistorLowDeviations[i]);
  }

}

// ================================================================
// ===               PHOTO TRANSISTORS LOOP                     ===
// ================================================================

void photoTransistorsLoop() {

  // grote if statement: als grenadeDetected false is gaat ge alles bekijken
  // met de high deviation en eventueel switchen naar true
  // TODO
  // dan gaat ge verzenden naar de hub dat de grenade detected is
  // en in deze if statement nog een if statement waar ge gaat kijken of er meer dan een minuut verstreken is met de timer
  // als dat zo is gaat ge nieuwe referenties nemen om drift te vermijden en reset ge de timer?

  if (!grenadeDetected) {
    for (int i=0; i<6; i++) {
      photoTransistorValues[i] = analogRead(photoTransistorPins[i]);
      if (photoTransistorValues[i] > photoTransistorReferences[i] + photoTransistorHighDeviations[i]
      || photoTransistorValues[i] < photoTransistorReferences[i] - photoTransistorHighDeviations[i]) {
        grenadeDetected = true;
        // deze lijn weghalen als het werkt:
        Serial.println("Grenade detected");
        break;
      }
    }

    // TODO

  }


  // grote else if statement: als grenadeDetected true is gaat ge alles bekijken
  // met de low deviation
  // dan gaat ge niks verzenden maar zet ge grenadeDetected terug op false

  else if (grenadeDetected) {
    for (int i=0; i<6; i++) {
      photoTransistorValues[i] = analogRead(photoTransistorPins[i]);
      if (photoTransistorValues[i] > photoTransistorReferences[i] + photoTransistorLowDeviations[i]
      || photoTransistorValues[i] < photoTransistorReferences[i] - photoTransistorLowDeviations[i]) {
        break;
      }
      grenadeDetected = false;
      // deze lijn weghalen als het werkt:
      Serial.println("Grenade no longer detected");
    }
  }

}


