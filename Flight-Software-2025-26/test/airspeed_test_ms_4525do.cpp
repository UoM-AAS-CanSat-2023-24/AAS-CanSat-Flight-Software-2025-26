#include <Arduino.h>
#include <Wire.h>

const byte SENSOR_ADDR = 0x28;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    delay(2000);

    Wire.begin();

    Serial.println("MS4525DO Test Starting...");
}

void loop() {

    // Request 4 bytes from sensor
    Wire.requestFrom(SENSOR_ADDR, (uint8_t)4);

    if (Wire.available() == 4) {

        byte b1 = Wire.read();
        byte b2 = Wire.read();
        byte b3 = Wire.read();
        byte b4 = Wire.read();

        // Extract status bits
        byte status = (b1 >> 6) & 0x03;

        // Extract 14-bit pressure
        uint16_t pressure_raw =
            ((b1 & 0x3F) << 8) | b2;

        // Extract 11-bit temperature
        uint16_t temp_raw =
            (b3 << 3) | (b4 >> 5);

        // Convert temperature to Celsius
        float temperature =
            ((float)temp_raw * 200.0 / 2047.0) - 50.0;

        Serial.print("Status: ");
        Serial.print(status);

        Serial.print("   Pressure Raw: ");
        Serial.print(pressure_raw);

        Serial.print("   Temp C: ");
        Serial.println(temperature);

        // Fast blink = successful read
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
    }
    else {

        Serial.println("Sensor read failed");

        // Slow blink = failed read
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
    }

    delay(500);
}