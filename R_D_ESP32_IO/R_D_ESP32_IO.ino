#include <esp_now.h>
#include <WiFi.h>

bool grenadeDetected = false;
const uint8_t photoTransistorPins[6] = {35,36,37,38,39,40};
uint16_t photoTransistorReferences[6] = {0,0,0,0,0,0};
uint16_t photoTransistorValues[6] = {0,0,0,0,0,0};
uint16_t photoTransistorHighDeviations[6] = {0,0,0,0,0,0};
uint16_t photoTransistorLowDeviations[6] = {0,0,0,0,0,0};

const uint8_t pressureSheetPins[4] = {1, 2, 3, 4};
uint16_t pressureSheetValues[4] = {0, 0, 0, 0};
uint16_t pressureSheetLowThresholds[4] = {0, 0, 0, 0};
uint16_t pressureSheetHighThresholds[4] = {0, 0, 0, 0};
bool pressureSheetToggle[4] = {false, false, false, false};

// choose the correct address for the hub
//uint8_t hubAddress[] = { 0xC4, 0x4F, 0x33, 0x41, 0x46, 0x99 };
//uint8_t hubAddress[] = { 0x84, 0xF7, 0x03, 0x88, 0xEC, 0xDA }; // s2 with the mac address on it
uint8_t hubAddress[] = {0xDC, 0xDA, 0x0C, 0x63, 0xCC, 0x9C};     // s3 number 2
esp_now_peer_info_t peerInfo;


// ================================================================
// ===          ON DATA SENT FUNCTION (ESP NOW)                 ===
// ================================================================

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ================================================================
// ===               SETUP FUNCTION                             ===
// ================================================================

void setup() {

  Serial.println("Setup");

  Serial.begin(115200);
  analogReadResolution(12);
  espNowSetup();
  pressureSheetsSetup();
  photoTransistorsSetup();

}

// ================================================================
// ===               LOOP FUNCTION                              ===
// ================================================================

void loop() {

  pressureSheetsLoop();
  photoTransistorsLoop();
  timerLoop();
  // ik weet niet of deze delay problemen gaat geven met de timer enzo
  // de delay stond eerst in de pressureSheetsLoop functie
  delay(5);

}

// ================================================================
// ===                  ESP NOW SETUP                           ===
// ================================================================

void espNowSetup() {

    WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //register first peer
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

}

// ================================================================
// ===                 PRESSURE SHEETS SETUP                    ===
// ================================================================

void pressureSheetsSetup() {

  for (int i = 0; i < 4; i++) {
    pressureSheetLowThresholds[i] = analogRead(pressureSheetPins[i]) / 3;
    pressureSheetHighThresholds[i] = analogRead(pressureSheetPins[i]) / 2;
    Serial.print("Pin ");
    Serial.print(pressureSheetPins[i]);
    Serial.print(": Low Threshold = ");
    Serial.print(pressureSheetLowThresholds[i]);
    Serial.print(", High Threshold = ");
    Serial.println(pressureSheetHighThresholds[i]);
  }

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
// ===                 PRESSURE SHEETS LOOP                     ===
// ================================================================

void pressureSheetsLoop() {

  for (int i = 0; i < 4; i++) {
    pressureSheetValues[i] = analogRead(pressureSheetPins[i]);
    if (pressureSheetToggle[i] && pressureSheetValues[i] > pressureSheetHighThresholds[i]) {
      Serial.print(pressureSheetPins[i]);
      Serial.println(" off");
      char message[10];
      snprintf(message, sizeof(message), "m/%d/%d", i + 1, 0);
      esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      while (result != ESP_OK) {
        result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      }
      pressureSheetToggle[i] = false;
    }
    else if (!pressureSheetToggle[i] && pressureSheetValues[i] < pressureSheetLowThresholds[i]) {
      Serial.print(pressureSheetPins[i]);
      Serial.println(" on");
      char message[10];
      snprintf(message, sizeof(message), "m/%d/%d", i + 1, 1);
      esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      while (result != ESP_OK) {
        result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      }
      pressureSheetToggle[i] = true;
    }
  }

}

// ================================================================
// ===               PHOTO TRANSISTORS LOOP                     ===
// ================================================================

void photoTransistorsLoop() {

  // grote if statement: als grenadeDetected false is gaat ge alles bekijken
  // met de high deviation en eventueel switchen naar true
  // dan gaat ge verzenden naar de hub dat de grenade detected is

  if (!grenadeDetected) {
    for (int i=0; i<6; i++) {
      photoTransistorValues[i] = analogRead(photoTransistorPins[i]);
      if (photoTransistorValues[i] > photoTransistorReferences[i] + photoTransistorHighDeviations[i]
      || photoTransistorValues[i] < photoTransistorReferences[i] - photoTransistorHighDeviations[i]) {
        grenadeDetected = true;
        Serial.println("Grenade detected");
        char message[10];
        message[0] = 'g';
        message[1] = '\0'; // Null-terminate the string
        esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        while (result != ESP_OK) {
          result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        }
        // break om ervoor te zorgen dat er slechts 1 keer verzonden wordt als meerdere sensoren tegelijkertijd de granaat detecteren
        break;
      }
    }
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
      Serial.println("Grenade no longer detected");
      return;
    }
  }

}

// ================================================================
// ===                     TIMER LOOP                           ===
// ================================================================

void timerLoop() {

  // TO DO
  // een if statement waar ge gaat kijken of er meer dan een minuut verstreken is met de timer
  // als dat zo is gaat ge nieuwe referenties nemen om drift te vermijden en reset ge de timer?

}

