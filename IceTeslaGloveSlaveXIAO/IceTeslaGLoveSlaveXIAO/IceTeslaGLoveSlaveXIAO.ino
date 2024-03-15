#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

// Set load cell pin number
int thumb_pin = 6;
int index_pin = 7;
int middle_pin = 8;
int ring_pin = 9;
int little_pin = 10;
int palm_pin = 1;

// Initialize servo code
// Define struct containing force sensed data
typedef struct force_data {
  int force_sensed_thumb = 0;
  int force_sensed_index = 0;
  int force_sensed_middle = 0;
  int force_sensed_ring = 0;
  int force_sensed_little = 0;
  int force_sensed_palm = 0;
  int buffer = 100;
};

// Initialize force_data struct
struct force_data posData;

void onRequest(){
  posData.force_sensed_thumb = analogRead(thumb_pin);
  posData.force_sensed_index = analogRead(index_pin);
  posData.force_sensed_middle = analogRead(middle_pin);
  posData.force_sensed_ring = analogRead(ring_pin);
  posData.force_sensed_little = analogRead(little_pin);
  
  
  Wire.write((uint8_t*)&posData, sizeof(posData));
}

void for_serial_print(){
  posData.force_sensed_thumb = analogRead(thumb_pin);
  posData.force_sensed_index = analogRead(index_pin);
  posData.force_sensed_middle = analogRead(middle_pin);
  posData.force_sensed_ring = analogRead(ring_pin);
  posData.force_sensed_little = analogRead(little_pin);

  Serial.print("Thumb: ");
  Serial.print(posData.force_sensed_thumb);
  Serial.print(" - Index: ");
  Serial.print(posData.force_sensed_index);
  Serial.print(" - Middle: ");
  Serial.print(posData.force_sensed_middle);
  Serial.print(" - Ring: ");
  Serial.print(posData.force_sensed_ring);
  Serial.print(" - Little: ");
  Serial.println(posData.force_sensed_little);

  delay(200);
}


void setup() {
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  Wire.onRequest(onRequest);

  analogReadResolution(12);

  Serial.println("Staring slave ...");

}

void loop() {
  
}
