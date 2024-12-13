//GIT version

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddressPrimGun[] = {0xDC, 0xDA, 0x0C, 0x64, 0x7F, 0xB8}; // send to esp32s3 divice 1 
// uint8_t broadcastAddressPrimGun[] = {0x24, 0xEC, 0x4A, 0x01, 0x32, 0xA0}; // prim gun 3
// uint8_t broadcastAddressPrimGun[] = {0xDC, 0xDA, 0x0C, 0x63, 0xCC, 0x9C}; // send to esp32s3 divice 2
String success;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println("\r\nDelivery Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Deliverd Successfully" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t *incomingData, int len) {
  // Serial.println("\r\nDataReceived: ");

  // Print data in hexadecimal format
  // Serial.print("Hex data: ");
  // for (int i = 0; i < len; i++) {
  //   Serial.print(incomingData[i], HEX);
  //   Serial.print(" ");
  // }
  
  // Print data as characters
  // Serial.print("\nASCII data: ");

  // Allocate memory for the message, including the null terminator
  char* message = new char[len + 1];

  for (int i = 0; i < len; i++) {
    message[i] = (char)incomingData[i];
    // Serial.print((char)incomingData[i]);
  }

  // Null-terminate the message to print as a string
  message[len] = '\0';

  Serial.println(message);

  // Free the allocated memory
  delete[] message;
}
 
void setup() {
  Serial.begin(1000000);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
 
  memcpy(peerInfo.peer_addr, broadcastAddressPrimGun, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void sendToPrimGun(String message){
  // const char message[] = "hello from esp 2"; // Define the message to send
  const char* messageData = message.c_str();
  int messageLength = message.length() + 1; // +1 to include the null terminator

  esp_err_t result = esp_now_send(broadcastAddressPrimGun, (uint8_t *) messageData, messageLength); // Send the message
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent Successfullt");
  // }
  // else {
  //   Serial.println("Geting Error while sending the data");
  // }
}
 
void loop() {
 if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n');
    sendToPrimGun(received);
 }
}