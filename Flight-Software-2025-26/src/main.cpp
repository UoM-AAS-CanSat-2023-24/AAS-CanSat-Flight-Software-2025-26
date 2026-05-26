
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C communication

  Serial.println("I2C Scanner");

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission(); // this will return 0 if a device is present at the address

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }


}