#include <Wire.h>

#define I2C_DEV_ADDR 0x55

uint16_t receivedData[5]; // Array to store received data

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Join the I2C bus as a master
}

void loop() {
  Wire.requestFrom(I2C_DEV_ADDR, sizeof(receivedData) * 2); // Request data from the slave

  // Wait until data is received
  while (Wire.available() < sizeof(receivedData) * 2) {
    delay(1);
  }

  // Read the received data into the array
  for (int i = 0; i < 5; i++) {
    receivedData[i] = Wire.read();
    receivedData[i] |= (Wire.read() << 8);
  }

  // Process the received data as needed
  for (int i = 0; i < 5; i++) {
    Serial.print("Force Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(receivedData[i]);
  }

  delay(1000); // Adjust delay as needed
}
