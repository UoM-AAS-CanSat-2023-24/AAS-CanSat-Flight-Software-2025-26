// sd_test.ino
// Simple SD card write test for Teensy 4.1
//
// What this does:
//   - Initialises the Teensy 4.1 built-in SD card slot
//   - Writes a dummy telemetry row once per second
//   - Prints status to USB serial so you can watch progress
//
// No external wiring needed - uses the built-in SD slot on the Teensy 4.1

#include <SD.h>
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#define LOG_FILENAME "sd_test.csv"

uint32_t packet_count = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // wait for USB serial

    Serial.println("=== SD card test ===");
    Serial.print("Initialising SD card... ");

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("FAILED. Check card is inserted.");
        while (true); // halt
    }
    Serial.println("OK");

    // Write a CSV header so the file is easy to open in Excel/Sheets
    File f = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!f) {
        Serial.println("Could not open file for writing. Halting.");
        while (true);
    }
    f.println("PACKET,TIME_MS,ALTITUDE_M,TEMPERATURE_C,PRESSURE_KPA,VOLTAGE_V,NOTE");
    f.close();

    Serial.print("Log file created: ");
    Serial.println(LOG_FILENAME);
    Serial.println("Writing 1 row/sec. Press reset to stop.");
    Serial.println();
}

void loop() {
    File f = SD.open(LOG_FILENAME, FILE_WRITE);

    if (!f) {
        Serial.println("ERROR: could not open file");
        delay(1000);
        return;
    }

    // Dummy data row - replace these with real sensor reads later
    char row[128];
    snprintf(row, sizeof(row),
        "%lu,%lu,%.1f,%.1f,%.1f,%.2f,DUMMY",
        packet_count,
        millis(),
        0.0f,   // altitude
        20.0f,  // temperature
        101.3f, // pressure kPa
        7.4f    // voltage
    );

    f.println(row);
    f.close();

    Serial.println(row); // mirror to USB serial so you can watch without removing the card
    packet_count++;
    delay(1000);
}
