#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>


MPU6050 mpu;
#define INTERRUPT_PIN 47  // W Set the interrupt pin
const int trigger_pin = 48;
int lastTriggerState = 1;


//ESP now 
uint8_t broadcastAddress[] = {0xDC, 0xDA, 0x0C, 0x63, 0xCC, 0x9C}; // send to esp32s3 divice 2
String success;

esp_now_peer_info_t peerInfo;


// Define custom SPI pins for ESP32-S3
#define SS_PIN 8       // Chip Select pin
#define RST_PIN 3      // Reset pin
#define SCK_PIN 18     // SPI Clock pin
#define MOSI_PIN 11    // Master Out Slave In pin
#define MISO_PIN 13    // Master In Slave Out pin

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
// MIFARE Ultralight has 12 blocks (0-11)
const byte blockStart = 5; // Start reading from block 4 (User data starts here)
const byte blockCount = 1; // Number of blocks to read


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  //-----------------MPU-----------------
  Wire.begin(20, 21); // SDA:GPIO 20 and SCL:GPIO 21
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);

  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);  // W Setup the interrupt pin as a input
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(83);  // last set 18/10/2024 using uduino_zero script
  mpu.setYGyroOffset(26);  //
  mpu.setZGyroOffset(73);
  mpu.setZAccelOffset(1521);  // W

  if (devStatus == 0) {
    // W Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    // W enable Arduino interrupt detection
    digitalPinToInterrupt(INTERRUPT_PIN);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    Serial.println("Error!");
  }

  //ESP now
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent); // set wich func to execute on data sent event
 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // set wich func to execute on data recieve event

  //-----------------Trigger-----------------
  pinMode(trigger_pin, INPUT_PULLUP);

  //-----------------mags(RFID)-----------------
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);  // Start SPI with custom pins
  mfrc522.PCD_Init();                    // Initialize MFRC522

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  //-----------------MPU-----------------
  // MPU();

  //-----------------Trigger-----------------
  // Trigger();

  //-----------------mags(RFID)-----------------
  RFID();

}


// ================================================================
// ===                     RFID FUNCTIONS                       ===
// ================================================================


void RFID(){
  String magData =  ReadMrfc522();
  if (magData != NULL){
    SendRfidEspNow(magData);
  }
}

void SendRfidEspNow(String data){
  String message = "mg/" + data;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) message.c_str(), message.length() + 1);

  if (result == ESP_OK) {
    Serial.println("RFID data sent successfully");
  } else {
    Serial.println("Error sending RFID data");
  }
}

String ReadMrfc522(){
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Card detected!");
    
    // Read data from the tag
    String data = readMultipleBlocks();

    mfrc522.PICC_HaltA();  // Halt the card
    return data;
  }else 
  {
    return "";
  }
  
}

// Function to read multiple blocks from the MIFARE Ultralight tag
String readMultipleBlocks() {
  String completePayload = "";

  // Read each block starting from blockStart up to blockStart + blockCount
  for (byte block = blockStart; block < blockStart + blockCount; block++) {
    byte buffer[18];       // Buffer to hold read data
    byte bufferLength = 18; // Length of the buffer

    // Read the block
    MFRC522::StatusCode status = mfrc522.MIFARE_Read(block, buffer, &bufferLength);
    if (status == MFRC522::STATUS_OK) {
      // Debug: print the raw block data
      Serial.print("Block ");
      Serial.print(block);
      Serial.print(": ");
      for (byte i = 0; i < bufferLength; i++) {
        Serial.print("0x");
        if (buffer[i] < 0x10) {
          Serial.print("0");
        }
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      // Append the readable characters from the block to the completePayload
      for (byte i = 0; i < bufferLength; i++) {
        if (buffer[i] >= 0x20 && buffer[i] <= 0x7E) { // Filter for readable ASCII characters
          completePayload += (char)buffer[i];
        }
      }
    } else {
      Serial.print("Failed to read block ");
      Serial.println(block);
      return "";
    }
  }

  Serial.print("Complete Payload: ");
  Serial.println(completePayload);

  // Extract specific parts from the payload
  String extractedData = extractStringByRange(completePayload, '/', 1, 3); // Read 2nd part till 4th
  Serial.print("Selected Data: ");
  Serial.println(extractedData);

  return extractedData;
}

// Function to split a string by a delimiter and return specified range of parts
String extractStringByRange(String data, char delimiter, int start, int end) {
  int partIndex = 0;          // Track which part we're on
  int startIdx = 0;           // Start index of the current part
  int endIdx = 0;             // End index of the current part
  String result = "";         // To store the extracted result
  bool firstPartAdded = false; // Flag to handle adding the delimiter correctly
  
  // Loop through each character in the data string
  for (int i = 0; i <= data.length(); i++) {
    // Check if we reached a delimiter or end of the string
    if (data[i] == delimiter || i == data.length()) {
      endIdx = i;
      
      // If the current part is within the requested range, add it to the result
      if (partIndex >= start && partIndex <= end) {
        if (firstPartAdded) {
          result += delimiter;  // Add the delimiter only between parts, not at the start
        }
        result += data.substring(startIdx, endIdx);
        firstPartAdded = true;
      }

      // Move to the next part
      partIndex++;
      startIdx = i + 1;
    }
  }

  return result;
}



// ================================================================
// ===                   TRIGGER FUNCTIONS                      ===
// ================================================================


void Trigger(){
  int triggerState = digitalRead(trigger_pin);
  if (triggerState == 0 || (lastTriggerState == 0 && triggerState == 1)) {
    lastTriggerState = triggerState;
    SendTriggerStateEspNow(triggerState);
  }
}

// send button state via ESP-NOW
void SendTriggerStateEspNow(int State) {
  String message = "tr/" + String(State);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) message.c_str(), message.length() + 1);

  if (result == ESP_OK) {
    Serial.println("Trigger state sent successfully");
  } else {
    Serial.println("Error sending trigger state");
  }
}


// ================================================================
// ===                   ESP now functions                      ===
// ================================================================

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nDelivery Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Deliverd Successfully" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t *incomingData, int len) {
  Serial.print("\r\nDataReceived: ");

  // Print data in hexadecimal format
  Serial.print("Hex data: ");
  for (int i = 0; i < len; i++) {
    Serial.print(incomingData[i], HEX);
    Serial.print(" ");
  }
  
  // Print data as characters
  Serial.print("\nASCII data: ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)incomingData[i]);
  }
  Serial.println();
}


// ================================================================
// ===                       MPU FUNCTIONS                      ===
// ================================================================


void MPU(){
  if (!dmpReady) {
    Serial.println("IMU not connected.");
    delay(10);
    return;
  }

  int mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {  // check if overflow
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // SendQuaternion();
    SendQuaternionEspNow();
    //SendEuler();
    //SendYawPitchRoll();
    //SendRealAccel();
    //SendWorldAccel();
  }
}

void SendQuaternionEspNow() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  String message = "r/" + String(q.w, 4) 
                  + "/" + String(q.x, 4) 
                  + "/" + String(q.y, 4) 
                  + "/" + String(q.z, 4);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) message.c_str(), message.length() + 1); // Send the message
  
  if (result == ESP_OK) {
    Serial.println("Quaternion data sent successfully");
  } else {
    Serial.println("Error sending quaternion data");
  }
}

// void SendQuaternion() {
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   Serial.print("r/");
//   Serial.print(q.w, 4);
//   Serial.print("/");
//   Serial.print(q.x, 4);
//   Serial.print("/");
//   Serial.print(q.y, 4);
//   Serial.print("/");
//   Serial.println(q.z, 4);
// }

// void SendEuler() {
//   // display Euler angles in degrees
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetEuler(euler, &q);
//   Serial.print(euler[0] * 180 / M_PI);
//   Serial.print("/");
//   Serial.print(euler[1] * 180 / M_PI);
//   Serial.print("/");
//   Serial.println(euler[2] * 180 / M_PI);
// }

// void SendYawPitchRoll() {
//   // display Euler angles in degrees
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//   Serial.print(ypr[0] * 180 / M_PI);
//   Serial.print("/");
//   Serial.print(ypr[1] * 180 / M_PI);
//   Serial.print("/");
//   Serial.println(ypr[2] * 180 / M_PI);
// }

// void SendRealAccel() {
//   // display real acceleration, adjusted to remove gravity
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetAccel(&aa, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//   Serial.print("a/");
//   Serial.print(aaReal.x);
//   Serial.print("/");
//   Serial.print(aaReal.y);
//   Serial.print("/");
//   Serial.println(aaReal.z);
// }

// void SendWorldAccel() {
//   // display initial world-frame acceleration, adjusted to remove gravity
//   // and rotated based on known orientation from quaternion
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetAccel(&aa, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//   mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//   Serial.print("a/");
//   Serial.print(aaWorld.x);
//   Serial.print("/");
//   Serial.print(aaWorld.y);
//   Serial.print("/");
//   Serial.println(aaWorld.z);
// }
