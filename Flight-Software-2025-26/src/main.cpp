// telemetry_test.ino
// Minimal test for Teensy 4.1 <-> XBee Pro S2C <-> Ground Station
//
// What this does:
//   - Sends a hardcoded dummy telemetry packet out Serial3 at 1Hz
//   - Also mirrors it to Serial (USB) so you can watch it on the Arduino serial monitor
//   - No sensors, no libraries, no dependencies - just UART
//
// Wiring:
//   Teensy TX6 (pin 24) -> XBee DIN
//   Teensy RX6 (pin 25) -> XBee DOUT
//   XBee VCC -> 3.3V, XBee GND -> GND
//
// XBee must already be configured with:
//   Baud: 115200, NETID: 1079, API mode: transparent (AT mode)
//   Both the CanSat XBee and ground station XBee need matching NETID

#include <Arduino.h>
#include <stdint.h>

#define TEAM_ID "1079"
#define XBEE_BAUD 115200


uint32_t packet_count = 0;

void setup() {
  Serial.begin(115200);   // USB - for monitoring on your PC
  Serial6.begin(XBEE_BAUD); // XBee

  // Wait for USB serial to come up (optional, remove for standalone testing)
  while (!Serial && millis() < 3000); // wait up to 3 seconds for USB serial connection

  Serial.println("=== Telemetry link test starting ===");
  Serial.println("Sending 1 packet/sec. Check ground station for received packets.");
  Serial.println();
}


void loop() {
    char packet[256];

    // Dummy packet in competition telemetry format:
    // TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE,
    // ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT,
    // GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
    // GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
    // CMD_ECHO
    snprintf(packet, sizeof(packet),
        "%s,00:00:%02lu,%lu,F,PRE_LAUNCH,"
        "0.0,20.0,101.3,"
        "7.4,0.50,"
        "0.0,0.0,0.0,0.0,0.0,9.8,"
        "00:00:00,0.0,0.0000,0.0000,0,"
        "NONE\n",
        TEAM_ID,
        (unsigned long)(millis() / 1000) % 60, // MISSION_TIME (dummy value, cycles 00:00:00 to 00:00:59)
        packet_count
    );

    Serial6.print(packet);  // -> XBee -> ground station
    Serial.print(packet);   // -> USB serial monitor (so you can verify locally too)

    packet_count++;
    delay(1000); // 1Hz
}
