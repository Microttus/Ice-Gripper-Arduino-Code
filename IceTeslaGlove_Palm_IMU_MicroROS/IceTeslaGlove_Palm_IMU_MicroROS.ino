#include <micro_ros_arduino.h>
#include "TinyPICO.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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

#define I2C_DEV_ADDR 0x56

// Accelerometer values
Adafruit_MPU6050 mpu;
uint16_t receivedData[10]; // Array to store received data
unsigned long timeOutMillis = 30;


// Initialize servo code
// Define struct containing force sensed data
typedef struct imu_data_struct {
  float accX = 0;
  float accY = 0;
  float accZ = 0;
  float gyrX = 0;
  float gyrY = 0;
  float gyrZ = 0;
};

// Initialize imu data struct
struct imu_data_struct imu_data;

void error_loop(){
  while(1){
    tp.DotStar_SetPixelColor(255, 0, 0);
    delay(100);               
  }
}


void update_imu_values()
{
  //  /* Get a new normalized sensor event */
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  imu_data.accX = accel.acceleration.x;
  imu_data.accY = accel.acceleration.y;
  imu_data.accZ = accel.acceleration.z;
  imu_data.gyrX = gyro.gyro.x;
  imu_data.gyrY = gyro.gyro.y;
  imu_data.gyrZ = gyro.gyro.z;

}

void for_serial_print(){
  Serial.print("IMU data -> AccX: ");
  Serial.print(imu_data.accX);
  Serial.print(" AccY: ");
  Serial.print(imu_data.accY);
  Serial.print(" AccZ: ");
  Serial.print(imu_data.accZ);
  Serial.print(" GyrX: ");
  Serial.print(imu_data.gyrX);
  Serial.print(" GyrY: ");
  Serial.print(imu_data.gyrY);
  Serial.print(" GyrZ: ");
  Serial.print(imu_data.gyrZ);
}

void setup() {
  Serial.begin(115200);
  //Wire.begin(); // Join the I2C bus as a master

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    error_loop();
  } else {
    Serial.println("MPU initiated");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  // Status green for start
  tp.DotStar_SetPixelColor(198, 115, 255);

  // Connect to wifi
  set_microros_wifi_transports("JM2_nett", "autyn33369", "192.168.0.107", 8888);

  delay(2000);


  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "glove_rotation_id1_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "ice_glove_palm_rotation_id1"));
  
}

void loop() {
  unsigned long startMillis = millis();

  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);

  update_imu_values();

  // Write new message
  msg.linear.x = imu_data.accX;
  msg.linear.y = imu_data.accY;
  msg.linear.z = imu_data.accZ;
  msg.angular.x = imu_data.gyrX; 
  msg.angular.y = imu_data.gyrY;
  msg.angular.z = imu_data.gyrZ;


  // Publish message
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  //for_serial_print();

  delay(2);

  unsigned long endMillis = millis() - startMillis;

  //Serial.print(" | Loop time -> ");
  //Serial.println(endMillis);
}
