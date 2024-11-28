#include <esp_now.h>
#include <WiFi.h>

const uint8_t pressureSheetPins[4] = {1, 2, 3, 4};
uint16_t pressureSheetValues[4] = {0, 0, 0, 0};
uint16_t pressureSheetLowThresholds[4] = {0, 0, 0, 0};
uint16_t pressureSheetHighThresholds[4] = {0, 0, 0, 0};
bool pressureSheetToggle[4] = {false, false, false, false};

uint8_t hubAddress[] = { 0xC4, 0x4F, 0x33, 0x41, 0x46, 0x99 };
// uint8_t hubAddress[] = { 0xDC, 0xDA, 0x0C, 0x63, 0xCC, 0x9C };
esp_now_peer_info_t peerInfo;

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

void setup() {
  Serial.begin(115200);

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

  analogReadResolution(12);
  for (int i = 0; i < 4; i++) {
    pressureSheetLowThresholds[i] = analogRead(pressureSheetPins[i]) / 3;
    pressureSheetHighThresholds[i] = 2 * analogRead(pressureSheetPins[i]) / 3;
    Serial.print("Pin ");
    Serial.print(pressureSheetPins[i]);
    Serial.print(": Low Threshold = ");
    Serial.print(pressureSheetLowThresholds[i]);
    Serial.print(", High Threshold = ");
    Serial.println(pressureSheetHighThresholds[i]);
  }
}

void loop() {
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
  delay(5);
}
