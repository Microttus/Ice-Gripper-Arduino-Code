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
typedef struct servo_pos_data {
  float base = 0;
  float arm = 0;
  float forarm = 0;
  float tool = 0;
};

// Initialize force_data struct
struct servo_pos_data ArmData;

// Create servo objects
Servo servo_base;
Servo servo_arm;
Servo servo_forarm;
Servo servo_tool;
//Servo servo_little;

// Set servo pin number
int servo_pin_base = 33;//32;
int servo_pin_arm = 32;//33;
int servo_pin_forarm = 4;
int servo_pin_tool = 14;

// Initialize other auxiliary variables
int limit = 180;
int offset[5] = {0, 0, 0, 0, 0}; // Offset values [thumb, index, middle, 
int init_set[5] = {90, 70, 20, 0, 0}; // Offset values [thumb, index, middle, ring, little]

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
  servo_base.write(map(static_cast<int>(ArmData.base), 0, 180, offset[0], limit));
  servo_arm.write(map(static_cast<int>(ArmData.arm), 180, 0, offset[1], limit));
  int forarm_pos = map(static_cast<int>(ArmData.forarm), 0, 180, offset[2], limit) + 90;
  servo_forarm.write(forarm_pos);
  servo_tool.write(map(static_cast<int>(ArmData.tool), 0, 180, offset[3], limit));

  delay(6);

  Serial.println(forarm_pos);
  //Serial.println(servo_thumb_val);
}

void subscription_callback(const void *msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.print("Message heard: ");
  auto newPos = msg->linear;
  auto newPos2 = msg->angular;
  ArmData.base = static_cast<float>(newPos.x);
  ArmData.arm = static_cast<float>(newPos.y);
  ArmData.forarm = static_cast<float>(newPos.z);
  ArmData.tool = static_cast<float>(newPos2.x);

  
  ServoControl();
}

void setup() {
  Serial.begin(115200);

  servo_base.write(90);
  servo_arm.write(90);
  servo_forarm.write(90);
  servo_tool.write(90);

  set_microros_wifi_transports("JM2_nett", "autyn33369", "192.168.0.107", 8888);

  delay(2000);
  
  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "icearm_exe", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "icearm_pos"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Hardware setup
  // Attach the servos to their respective pins
  servo_base.attach(servo_pin_base); 
  servo_arm.attach(servo_pin_arm);
  servo_forarm.attach(servo_pin_forarm); 
  servo_tool.attach(servo_pin_tool);

  // Start-up sequence
  //servo_index.write(0);
  //servo_thumb.write(0);
  //servo_middle.write(0);
  //servo_ring.write(0);
  //delay(500);
  servo_base.write(90);
  servo_arm.write(90);
  servo_forarm.write(90);
  servo_tool.write(90);
  //delay(500);
  //servo_index.write(0);
  //servo_thumb.write(0);
  //servo_middle.write(0);
  //servo_ring.write(0);
  delay(1000);
}

void loop() {
  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);
  //delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
