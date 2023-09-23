#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Define struct containing force sensed data
typedef struct force_data {
  uint16_t force_sensed_index = 0;
  uint16_t force_sensed_thumb = 0;
} force_data;

// Initialize force_data struct
struct force_data forceData;

// Create servo objects
Servo servo_index;
Servo servo_thumb;

// Set servo pin number
int servo_pin_thumb = 4;
int servo_pin_index = 22;

// Define Master Mac address
uint8_t macMaster[] = {0x40, 0x22, 0xD8, 0x3C, 0x1C, 0xCC};

// Define peer info struct for first device
esp_now_peer_info_t peer1;

// Initialize other auxiliary variables
int limit = 180;
int offset_index = 0;
int offset_thumb = 0;

// Define the callback function connected to the event of receiving ESP-NOW data
void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&forceData, incomingData, sizeof(forceData));
  servo_index.write(map(forceData.force_sensed_index, 0, 255, offset_index, limit));
  servo_thumb.write(map(forceData.force_sensed_thumb, 0, 255, offset_thumb, limit));
}


void setup() {
// Start serial communication
Serial.begin(115200);

// Attach the servos to their respective pins
servo_index.attach(servo_pin_thumb); 
servo_thumb.attach(servo_pin_index);  

Serial.println("[INFO]: Servos atatched");
delay(2000);

// Disconnect from WiFi and erase the stored config
//esp_now_deinit();
WiFi.disconnect();

// Start up sequence
servo_index.write(0);
servo_thumb.write(0);
delay(500);
servo_index.write(50);
servo_thumb.write(50);
delay(500);
servo_index.write(0);
servo_thumb.write(0);

// Set device as a Wi-Fi Station
WiFi.mode(WIFI_STA);

// Init ESP-NOW
if (esp_now_init() != 0) {
    Serial.println("[ERROR] Failed to initialie ESP-NOW");
    return;
} else {
		Serial.println("[INFO] ESP-NOW Initialized");
}

// Set the device to act as both a sender and receiver using ESP-NOW
//esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

// Set peer1's address and channel, and disable encryption
    memcpy(peer1.peer_addr, macMaster, 6);
    peer1.channel = 1;
    peer1.encrypt = 0;

// Add the MAC address of the master device as a peer for ESP-NOW communication
esp_now_add_peer(&peer1);

// We can register the receiver callback function
esp_now_register_recv_cb(onDataReceiver);

}



void loop() {
  // Do nothing, but added delay do too good practice
  delay(1);
}
