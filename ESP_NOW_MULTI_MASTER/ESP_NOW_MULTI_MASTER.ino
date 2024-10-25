#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress1[] = { 0xC4, 0x4F, 0x33, 0x41, 0x46, 0x99 };
uint8_t broadcastAddress2[] = { 0x84, 0xF7, 0x03, 0x89, 0x5E, 0x50 };
uint8_t broadcastAddress3[] = { 0x84, 0xF7, 0x03, 0x88, 0xEC, 0xDA };

int x = 0, y = 0, z = 0;

esp_now_peer_info_t peerInfo;

// callback when data is sent
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

// Callback function for when data is received
void OnDataReceived(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Print MAC address of the sender
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Data received from: ");
  Serial.print(macStr);

  int receivedValue;
  memcpy(&receivedValue, incomingData, sizeof(receivedValue));

  // Compare the MAC address to identify the sender
  if (memcmp(mac_addr, broadcastAddress1, 6) == 0) {
    if (receivedValue == x) {
      Serial.println(" Correctly received");
    }
    else {
      Serial.println(" Incorrectly received");
    }
  }
  else if (memcmp(mac_addr, broadcastAddress2, 6) == 0) {
    if (receivedValue == y) {
      Serial.println(" Correctly received");
    }
    else {
      Serial.println(" Incorrectly received");
    }
  }
  else if (memcmp(mac_addr, broadcastAddress3, 6) == 0) {
    if (receivedValue == z) {
      Serial.println(" Correctly received");
    }
    else {
      Serial.println(" Incorrectly received");
    }
  }
  else {
    Serial.println(" Unknown slave.");
  }
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
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  //register second peer
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  //register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataReceived));
}

void loop() {
  x = random(0, 20);
  y = random(0, 20);
  z = random(0, 20);

  esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *)&x, sizeof(x));
  esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *)&y, sizeof(y));
  esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *)&z, sizeof(z));

  if ((result1 == ESP_OK) && (result2 == ESP_OK) && (result3 == ESP_OK)) {
    Serial.println("All sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}
