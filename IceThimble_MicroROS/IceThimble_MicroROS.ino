#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
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
struct force_data forceData;

// Create servo objects
Servo servo_index;
Servo servo_thumb;
Servo servo_middle;
Servo servo_ring;
//Servo servo_little;

// Set servo pin number
int servo_pin_thumb = 4;//32;
int servo_pin_index = 22;//33;
int servo_pin_middle = 27;
int servo_pin_ring = 15;

// Initialize other auxiliary variables
int limit = 240;
int offset[5] = {50, 70, 70, 70, 90}; // Offset values [thumb, index, middle, ring, little]

void error_loop(){
  while(1){
    tp.DotStar_SetPixelColor(255, 0, 0 );
    delay(100);
  }
}

void ServoControl()
{
  //Testing delay
  delay(2);
  servo_thumb.write(map(static_cast<int>(forceData.force_sensed_thumb), 0, 255, offset[0], limit));
  servo_index.write(map(static_cast<int>(forceData.force_sensed_index), 0, 255, offset[1], limit));
  servo_middle.write(map(static_cast<int>(forceData.force_sensed_middle), 0, 255, offset[2], limit));
  servo_ring.write(map(static_cast<int>(forceData.force_sensed_ring), 0, 255, offset[3], limit));

  delay(6);

  //Serial.println(forceData.force_sensed_thumb);
  //Serial.println(servo_thumb_val);
}

void subscription_callback(const void *msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.print("Message heard: ");
  auto newPos = msg->linear;
  auto newPos2 = msg->angular;
  forceData.force_sensed_thumb = static_cast<float>(newPos.x);
  forceData.force_sensed_index = static_cast<float>(newPos.y);
  forceData.force_sensed_middle = static_cast<float>(newPos.z);
  forceData.force_sensed_ring = static_cast<float>(newPos2.x);

  
  ServoControl();
}

void setup() {
  Serial.begin(115200);

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
    "finger_force"));

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
  servo_index.write(0);
  servo_thumb.write(0);
  //servo_middle.write(0);
  //servo_ring.write(0);
  delay(500);
  servo_index.write(offset[0]);
  servo_thumb.write(offset[1]);
  //servo_middle.write(offset[2]);
  //servo_ring.write(offset[3]);
  delay(500);
  servo_index.write(0);
  servo_thumb.write(0);
  //servo_middle.write(0);
  //servo_ring.write(0);
  delay(2000);
}

void loop() {
  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);
  //delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
