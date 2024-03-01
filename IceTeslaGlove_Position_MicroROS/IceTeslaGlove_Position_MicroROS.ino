#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include "TinyPICO.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Thread.h>
#include <WiFi.h>
#include <Wire.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Initial led object
TinyPICO tp = TinyPICO();

#define I2C_DEV_ADDR 0x55

uint16_t receivedData[6]; // Array to store received data
unsigned long timeOutMillis = 30;


// Initialize servo code
// Define struct containing force sensed data
typedef struct force_data {
  float force_sensed_index = 0;
  float force_sensed_thumb = 0;
  float force_sensed_middle = 0;
  float force_sensed_ring = 0;
  float force_sensed_little = 0;
  float force_sensed_palm = 0;
};

// Initialize force_data struct
struct force_data posData;

// Initialize other auxiliary variables
int limit = 254;
int offset[6] = {0, 0, 0, 0, 0, 0}; // Offset values [thumb, index, middle, ring, little, palm]

void error_loop(){
  while(1){
    tp.DotStar_SetPixelColor(255, 0, 0);
    delay(100);
  }
}


void read_volt()
{
  Serial.print("Started req");
  Wire.requestFrom(I2C_DEV_ADDR, sizeof(receivedData) * 2); // Request data from the slave

  // Wait until data is received
  unsigned long startTime = millis();
  while (Wire.available() < sizeof(receivedData) * 2) {
    if((millis() - startTime) > timeOutMillis) {
      Serial.println(" - ! -");
      Wire.endTransmission();
      return;
    }
    delay(1);
  }

  Serial.print(" -- ");
  // Read the received data into the array
  for (int i = 0; i < 6; i++) {
    receivedData[i] = Wire.read();
    receivedData[i] |= (Wire.read() << 8);
  }

  posData.force_sensed_thumb = receivedData[0];
  posData.force_sensed_index = receivedData[2];
  posData.force_sensed_middle = receivedData[4];
  posData.force_sensed_ring = receivedData[6];
  posData.force_sensed_little = receivedData[8];

  //Serial.print(posData.force_sensed_thumb);
  //Serial.println(analogRead(thumb_pin));
  Serial.println("Finished req");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Join the I2C bus as a master

  // Connect to wifi
  set_microros_wifi_transports("JM2_nett", "autyn33369", "192.168.0.107", 8888);

  delay(2000);
  
  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "tinypico_finger_position", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "ice_glove_pos_id1"));
  
}

void loop() {
  unsigned long startMillis = millis();

  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);

  // Updater finger values
  //if(voltReadThread.shouldRun()){
   // voltReadThread.run();
  //}
  read_volt();

  // Write new message
  msg.linear.x = posData.force_sensed_thumb;
  msg.linear.y = posData.force_sensed_index;
  msg.linear.z = posData.force_sensed_middle;
  msg.angular.x = posData.force_sensed_ring;
  msg.angular.y = posData.force_sensed_little;
  msg.angular.z = 42;

  // Publish message
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  //Serial.println(analogRead(thumb_pin));
  Serial.print(" --- ");
  //Serial.println(msg.linear.y);

  unsigned long endMillis = millis() - startMillis;

  Serial.println(endMillis);
  delay(50);
}
