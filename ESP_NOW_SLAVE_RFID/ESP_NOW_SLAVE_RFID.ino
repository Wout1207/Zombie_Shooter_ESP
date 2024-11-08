#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN 4    // RST pin for RC522
#define SS_PIN 21    // SDA/SS pin for RC522

MFRC522 rfid(SS_PIN, RST_PIN);

// Buffer to hold RFID tag data
byte tagData[10];  // Adjust size based on tag length, up to 10 bytes

int received = 0;

uint8_t masterAddress[] = { 0xEC, 0xDA, 0x3B, 0x8E, 0xCD, 0x98 };

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
  Serial.println(macStr);
  // Not doing anything with this as of now
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  SPI.begin();
  rfid.PCD_Init();
  Serial.println("Scan RFID tag");
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //register first peer
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataReceived));
}
 
void loop() {
  // Check for a new RFID card
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Tag ID:");
    
    // Copy RFID tag to buffer
    for (byte i = 0; i < rfid.uid.size; i++) {
      tagData[i] = rfid.uid.uidByte[i];
      Serial.print(tagData[i] < 0x10 ? " 0" : " ");
      Serial.print(tagData[i], HEX);
    }
    Serial.println();

    // Send RFID tag data to the master
    esp_err_t result = esp_now_send(masterAddress, tagData, rfid.uid.size);
    if (result == ESP_OK) {
      Serial.println("Tag sent successfully");
    } else {
      Serial.println("Error sending tag");
    }

    // Halt the current tag so we can detect a new one in the future
    rfid.PICC_HaltA();
  }
  else {
    delay(50);
  }
}