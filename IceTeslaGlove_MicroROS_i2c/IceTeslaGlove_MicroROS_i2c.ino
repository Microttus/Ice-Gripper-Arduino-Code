#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "TinyPICO.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Initial led object
TinyPICO tp = TinyPICO();

// Define I2C ADDR
#define I2C_DEV_ADDR 0x55

// Define i2c variables
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

typedef struct servo_pos {
  int thumb = 0;
  int index = 0;
  int middle = 0;
  int ring = 0;
  int little = 0;
  int palm = 0;
};

// Initialize force_data struct
struct force_data forceData;
struct force_data posData;

struct servo_pos servoPos;
struct servo_pos forcePos;

// Initialize profile setting
//int load_cell_pos_profile[6][2] = {{3000, 1500}, {3400, 1500}, {2500, 1100}, {2500, 800}, {1, 0}, {1, 0}};
int load_cell_pos_profile[6][2] = {{180, 140}, {180, 110}, {200, 150}, {200, 130}, {160, 100}, {42, 0}};
int proto_1[8] = {50, 130, 30, 110, 250, 140, 210, 160};
int proto_2[8] = {80, 120, 120, 60, 100, 160, 140, 60};
float comp_filter_const = 1;
float filter_in_sign = 0.1;

// Create servo objects
Servo servo_index;
Servo servo_thumb;
Servo servo_middle;
Servo servo_ring;
//Servo servo_little;

// Set servo pin number
int servo_pin_thumb = 33;//32-4;
int servo_pin_index = 32;//33-22;
int servo_pin_middle = 4;
int servo_pin_ring = 14;

// Initialize other auxiliary variables
int limit = 180;
int offset[6] = {0, 0, 0, 0, 0, 0}; // Offset values [thumb, index, middle, ring, little, palm]

void error_loop(){
  while(1){
    tp.DotStar_SetPixelColor(255, 0, 0);
    delay(100);
  }
}

void ReadFingerPos()
{
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

  // Read the received data into the array
  for (int i = 0; i < 10; i++) {
    receivedData[i] = Wire.read();
    receivedData[i] |= (Wire.read() << 8);
  }

  posData.force_sensed_thumb = comp_filter_f(constrain(receivedData[0], load_cell_pos_profile[0][0], load_cell_pos_profile[0][1]),posData.force_sensed_thumb);
  posData.force_sensed_index = comp_filter_f(constrain(receivedData[2], load_cell_pos_profile[1][1], load_cell_pos_profile[1][0]),posData.force_sensed_index);
  posData.force_sensed_middle = comp_filter_f(constrain(receivedData[4], load_cell_pos_profile[2][1], load_cell_pos_profile[2][0]),posData.force_sensed_middle);
  posData.force_sensed_ring = comp_filter_f(constrain(receivedData[6], load_cell_pos_profile[3][1], load_cell_pos_profile[3][0]),posData.force_sensed_ring);
  posData.force_sensed_little = comp_filter_f(constrain(receivedData[8], load_cell_pos_profile[4][1], load_cell_pos_profile[5][0]),posData.force_sensed_little);
  
  Serial.print(receivedData[6]);
  Serial.print(" -> ");
}

void finger_pos_to_servo()
{
  servoPos.thumb = comp_filter(map(static_cast<int>(posData.force_sensed_thumb), load_cell_pos_profile[0][1], load_cell_pos_profile[0][0], proto_2[0], proto_2[1]), servoPos.thumb);
  servoPos.index = comp_filter(map(static_cast<int>(posData.force_sensed_index), load_cell_pos_profile[1][1], load_cell_pos_profile[1][0], proto_2[3], proto_2[2]), servoPos.index);
  servoPos.middle = comp_filter(map(static_cast<int>(posData.force_sensed_middle), load_cell_pos_profile[2][1], load_cell_pos_profile[2][0], proto_2[5], proto_2[4]), servoPos.middle);
  servoPos.ring = comp_filter(map(static_cast<int>(posData.force_sensed_ring), load_cell_pos_profile[3][1], load_cell_pos_profile[3][0], proto_2[6], proto_2[7]), servoPos.ring);

  Serial.print(servoPos.ring);
  Serial.print(" - ");
  Serial.println(posData.force_sensed_ring);
}

void finger_force_compensation()
{
  servoPos.thumb = servoPos.thumb + forcePos.thumb;
  servoPos.index = servoPos.index + forcePos.index;
  servoPos.middle = servoPos.middle + forcePos.middle;
  servoPos.ring = servoPos.ring + forcePos.ring;
  servoPos.little = servoPos.little + forcePos.little;
}

int comp_filter(int in_data, int last_data)
{
  float in_float = static_cast<float>(in_data);
  float last_float = static_cast<float>(last_data);

  float return_float = (in_float*comp_filter_const)+(last_float*(1-comp_filter_const));
  int return_int = static_cast<int>(return_float);

  return return_int;
}

float comp_filter_f(float in_data, float last_data)
{
  return (in_data*filter_in_sign)+(last_data*(1-filter_in_sign));
}

void ServoControl()
{
  //Testing delay
  servo_thumb.write(map(static_cast<int>(forceData.force_sensed_thumb), 0, 255, offset[0], limit));
  servo_index.write(map(static_cast<int>(forceData.force_sensed_index), 0, 255, offset[1], limit));
  servo_middle.write(map(static_cast<int>(forceData.force_sensed_middle), 0, 255, offset[2], limit));
  servo_ring.write(map(static_cast<int>(forceData.force_sensed_ring), 0, 255, offset[3], limit));

  delay(5);

  Serial.println(forceData.force_sensed_thumb);
  //Serial.println(servo_thumb_val);
}

void ServoControlI2C()
{
  //Testing delay
  servo_thumb.write(servoPos.thumb);
  servo_index.write(servoPos.index);
  servo_middle.write(servoPos.middle);
  servo_ring.write(servoPos.ring);

  delay(5);

  Serial.println(forceData.force_sensed_thumb);
}

void ServoControlCal()
{
  //Testing delay
  servo_thumb.write(forcePos.thumb);
  servo_index.write(forcePos.index);
  servo_middle.write(forcePos.middle);
  servo_ring.write(forcePos.ring);

  delay(5);

  serial_print();
}

void subscription_callback(const void *msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.print("Message heard: ");
  auto newPos = msg->linear;
  auto newPos2 = msg->angular;

  forcePos.thumb = static_cast<int>(newPos.x);
  forcePos.index = static_cast<int>(newPos.y);
  forcePos.middle = static_cast<int>(newPos.z);
  forcePos.ring = static_cast<int>(newPos2.x);

  Serial.print(forcePos.thumb);

  //ServoControl();
  
}

void serial_print(){
  Serial.print(servoPos.thumb);
  Serial.print(" - ");
  Serial.print(servoPos.index);
  Serial.print(" - ");
  Serial.print(servoPos.middle);
  Serial.print(" - ");
  Serial.println(servoPos.ring);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  set_microros_wifi_transports("JM2_nett", "autyn33369", "192.168.0.107", 8888);

  delay(2000);
  
  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "tinypico_finger_force", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "ice_glove_id1"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Hardware setup
  // Attach the servos to their respective pins
  servo_thumb.attach(servo_pin_thumb); 
  servo_index.attach(servo_pin_index);
  servo_middle.attach(servo_pin_middle); 
  servo_ring.attach(servo_pin_ring);

  // Start up sequence
  servo_index.write(90);
  servo_thumb.write(90);
  servo_middle.write(90);
  servo_ring.write(90);
  delay(500);
  servo_index.write(70);
  servo_thumb.write(70);
  servo_middle.write(70);
  servo_ring.write(70);
  delay(500);
  servo_index.write(90);
  servo_thumb.write(90);
  servo_middle.write(90);
  servo_ring.write(90);
  delay(2000);

  forcePos.thumb = 90;
  forcePos.index = 90;
  forcePos.middle = 90;
  forcePos.ring = 90;
}

void loop() {
  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);

  //delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Set servo poositions
  //ServoControl();

  // I2C implimentation
  ReadFingerPos();
  finger_pos_to_servo();
  //finger_force_compensation();
  ServoControlI2C();

  //ServoControlCal();
  serial_print();

}
