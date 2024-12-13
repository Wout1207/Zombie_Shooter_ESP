#include <esp_now.h>
#include <WiFi.h>
#include <esp_timer.h>
#define CONVERSIONS_PER_PIN 35
#define LED_PIN 37

// Variables for the grenades
bool grenadeThrown = false;
bool grenadeDetected[] = {false, false, false, false, false, false};
uint8_t photoTransistorPins[] = {1,2,3,4,5,6,7,8,9,10};
uint16_t photoTransistorReferences[] = {0,0,0,0,0,0};
uint16_t photoTransistorValues[] = {0,0,0,0,0,0};
uint16_t photoTransistorHighDeviations[] = {0,0,0,0,0,0};
uint16_t photoTransistorLowDeviations[] = {0,0,0,0,0,0};

// Variables for the movement sheets
const uint8_t pressureSheetPins[] = {7,8,9,10};
uint16_t pressureSheetValues[] = {0, 0, 0, 0};
uint16_t pressureSheetLowThresholds[] = {0, 0, 0, 0};
uint16_t pressureSheetHighThresholds[] = {0, 0, 0, 0};
bool pressureSheetToggle[] = {false, false, false, false};

// Variables for adc
// Calculate how many pins are declared in the array - needed as input for the setup function of ADC Continuous
uint8_t adc_pins_count = 10;
// Flag which will be set in ISR when conversion is done
volatile bool adc_coversion_done = false;
// Result structure for ADC Continuous reading
adc_continuous_data_t *result = NULL;
// ISR Function that will be triggered when ADC conversion is done
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

// Variables for led strip
esp_timer_handle_t myTimer; // Declare the timer handle globally
bool isLedOn = true;
// Timer callback function
void onTimer(void* arg) {
  esp_timer_stop(myTimer);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Timer stopped.");
}

// choose the correct address for the hub
//uint8_t hubAddress[] = { 0xC4, 0x4F, 0x33, 0x41, 0x46, 0x99 };
// uint8_t hubAddress[] = { 0x98, 0x3D, 0xAE, 0xEC, 0xCB, 0x5C };
//uint8_t hubAddress[] = { 0x84, 0xF7, 0x03, 0x88, 0xEC, 0xDA }; // s2 with the mac address on it
// uint8_t hubAddress[] = {0xDC, 0xDA, 0x0C, 0x63, 0xCC, 0x9C};     // s3 number 2
uint8_t hubAddress[] = {0x24, 0xEC, 0x4A, 0x01, 0x32, 0xA0};     // s3 number 3
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
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Configure and initialize the timer
  esp_timer_create_args_t timerConfig = {
      .callback = &onTimer, // Function to call when the timer triggers
      .arg = nullptr,       // Optional argument (unused here)
      .name = "FixedTimer"  // Name for debugging
  };
  // Create the timer
  esp_timer_create(&timerConfig, &myTimer);
  
  espNowSetup();
  // timerSetup();
  adcSetup();
  while (!adc_coversion_done) ;
  adc_coversion_done = false;
  setReferences();
}

// ================================================================
// ===               LOOP FUNCTION                              ===
// ================================================================

void loop() {
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    photoTransistorsLoop();
  }
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
// ===                       ADC SETUP                          ===
// ================================================================

void adcSetup() {
  // Optional for ESP32: Set the resolution to 9-12 bits (default is 12 bits)
  analogContinuousSetWidth(12);
  // Optional: Set different attenaution (default is ADC_11db)
  analogContinuousSetAtten(ADC_11db);
  // Setup ADC Continuous with following input:
  // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
  analogContinuous(photoTransistorPins, adc_pins_count, CONVERSIONS_PER_PIN, 83333, &adcComplete); //20000
  // Start ADC Continuous conversions
  analogContinuousStart();
}

// ================================================================
// ===              REFERENCE SETTING FUNCTION                  ===
// ================================================================

void setReferences() {
  if (analogContinuousRead(&result, 0)) {
    analogContinuousStop();
    for (int i = 0; i < 6; i++) {
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
    for (int i = 0; i < 4; i++) {
      pressureSheetLowThresholds[i] = result[i + 6].avg_read_raw / 3;
      pressureSheetHighThresholds[i] = pressureSheetLowThresholds[i] * 2;
      Serial.print("Pin ");
      Serial.print(pressureSheetPins[i]);
      Serial.print(": Low Threshold = ");
      Serial.print(pressureSheetLowThresholds[i]);
      Serial.print(", High Threshold = ");
      Serial.println(pressureSheetHighThresholds[i]);
    }
    analogContinuousStart();
  } else {
    Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
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
      // char message[10];
      // snprintf(message, sizeof(message), "m/%d/%d", i + 1, 0);
      // esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      // while (result != ESP_OK) {
      //   result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      // }
      pressureSheetToggle[i] = false;
    }
    else if (!pressureSheetToggle[i] && pressureSheetValues[i] < pressureSheetLowThresholds[i]) {
      Serial.print(pressureSheetPins[i]);
      Serial.println(" on");
      // char message[10];
      // snprintf(message, sizeof(message), "m/%d/%d", i + 1, 1);
      // esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      // while (result != ESP_OK) {
      //   result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      // }
      pressureSheetToggle[i] = true;
    }
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
    for (int i = 0; i < adc_pins_count - 4; i++) {
      photoTransistorValues[i] = result[i].avg_read_raw;
      if (!grenadeDetected[i]) {
        if (photoTransistorValues[i] > photoTransistorReferences[i] + photoTransistorHighDeviations[i] ||
            photoTransistorValues[i] < photoTransistorReferences[i] - photoTransistorHighDeviations[i]) {
          Serial.print("Pin: ");
          Serial.print(photoTransistorPins[i]);
          Serial.print(" Value: ");
          Serial.print(photoTransistorValues[i]);
          Serial.println(" Grenade detected");
          grenadeDetected[i] = true;
        }
      }
      else {
        if (photoTransistorValues[i] < photoTransistorReferences[i] + photoTransistorLowDeviations[i] &&
            photoTransistorValues[i] > photoTransistorReferences[i] - photoTransistorLowDeviations[i]) {
          Serial.print("Pin: ");
          Serial.print(photoTransistorPins[i]);
          Serial.print(" Value: ");
          Serial.print(photoTransistorValues[i]);
          Serial.println(" No Grenade detected");
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
      char message[10];
      snprintf(message, sizeof(message), "g");
      esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
      digitalWrite(LED_PIN, LOW);
      // esp_timer_start_once(myTimer, 100000);
      Serial.println("Grenade thrown! Message sent.");
    }
    // Reset grenadeThrown to false if all grenades are cleared
    if (grenadeThrown && allGrenadesCleared) {
      grenadeThrown = false;
      digitalWrite(LED_PIN, LOW);
      esp_timer_start_once(myTimer, 100000);
      Serial.println("Grenades cleared.");
    }

    for (int i = 6; i < adc_pins_count; i++) {
      pressureSheetValues[i-6] = result[i].avg_read_raw;
      if (pressureSheetToggle[i-6] && pressureSheetValues[i-6] > pressureSheetHighThresholds[i-6]) {
        Serial.print(pressureSheetPins[i-6]);
        Serial.println(" off");
        char message[10];
        snprintf(message, sizeof(message), "m/%d/%d", i - 6 + 1, 0);
        esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        while (result != ESP_OK) {
          result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        }
        pressureSheetToggle[i-6] = false;
      }
      else if (!pressureSheetToggle[i-6] && pressureSheetValues[i-6] < pressureSheetLowThresholds[i-6]) {
        Serial.print(pressureSheetPins[i-6]);
        Serial.println(" on");
        char message[10];
        snprintf(message, sizeof(message), "m/%d/%d", i-6 + 1, 1);
        esp_err_t result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        while (result != ESP_OK) {
          result = esp_now_send(hubAddress, (uint8_t *)message, strlen(message));
        }
        pressureSheetToggle[i-6] = true;
      }
    }
    
    analogContinuousStart();
  } else {
    Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
  }
}

// ================================================================
// ===                     TIMER LOOP                           ===
// ================================================================

// void resetReferences() {
//   Serial.println("Reset reference function");
//   if (!grenadeDetected) {
//     // Reset reference of grenades
//     photoTransistorsSetup();
//   }
//   for (int i = 0; i < 4; i++) {
//     if (pressureSheetToggle[i]) return;
//   }
//   // Reset references of pressure sheets
//   pressureSheetsSetup();
// }

