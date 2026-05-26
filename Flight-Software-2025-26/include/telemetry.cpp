#pragma once 
#include <Arduino.h>
#include <XBee.h>
#include "sensing.cpp" // needed for GPS-based stuff

// Team ID for CanSat 2026 - Team 1079
#define TEAM_ID "1079"
 
// Telemetry output via Serial3 (XBee Pro S2C connected here per PCB layout)
// Baud rate 115200 to match XBee Pro S2C config

char received_telemetry_raw[60] = {};

bool telemetry_setup(){
    // Initialize Serial communication for telemetry
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    return true; // telemetry setup successful
}

// -------------------------------------------------------
// SENSOR INDEX MAP (sensor_readings[] array)
// These match the indices used in sensing.cpp - update if
// sensing.cpp changes its array layout.
// -------------------------------------------------------
// [0]  altitude_baro   - barometric altitude (m, relative to ground, from Matek F405 SPL06-001)
// [1]  temperature     - LM335AZ temperature (°C)
// [2]  pressure        - barometric pressure (Pa, /1000 → kPa in telemetry)
// [3]  voltage         - INA260 bus voltage (mV, /1000 → V in telemetry)
// [4]  current         - INA260 bus current (mA, /1000 → A in telemetry)
// [5]  gyro_r          - ICM-20948 gyro roll   (°/s, via Matek F405)
// [6]  gyro_p          - ICM-20948 gyro pitch  (°/s)
// [7]  gyro_y          - ICM-20948 gyro yaw    (°/s)
// [8]  accel_r         - ICM-20948 accel roll  (m/s²)
// [9]  accel_p         - ICM-20948 accel pitch (m/s²)
// [10] accel_y         - ICM-20948 accel yaw   (m/s²)
// [11] airspeed        - MS4525DO airspeed (Pa, raw differential pressure)
// [12] bus_power       - INA260 power (mW, /1000 → W in telemetry)
// [13] main_soc        - battery state of charge (%, estimated)
// GPS fields are passed separately (see send_telemetry signature below)
// NOTE: Array is 14 elements - bounds checked before access
 
// -------------------------------------------------------
// FSW FLAGS BYTE (fsw_flags_arg) - bit definitions
// -------------------------------------------------------
// Bit 7 (0x80): Mode       1 = Simulation, 0 = Flight
// Bit 6 (0x40): Payload release mechanism state  1 = RELEASED, 0 = LATCHED
// Bit 5 (0x20): Egg (nose cone) release state    1 = RELEASED, 0 = LATCHED
// Bit 4 (0x10): Telemetry TX enable              1 = TX on, 0 = TX off
// Bit 3 (0x08): Camera 1 active (payload release ESP32S3)
// Bit 2 (0x04): Camera 2 active (ground/egg ESP32S3)
// Bit 1 (0x02): Matek F405 online
// Bit 0 (0x01): Spare


// -------------------------------------------------------
// SEND TELEMETRY
// Produces one CSV telemetry packet per competition spec:
//   TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE,
//   ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT,
//   GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
//   GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
//   CMD_ECHO,
//   [optional] SUBSTATE, MAIN_SOC, BUS_POWER, AIR_SPEED,
//              ACTIVE_MECHS, ACTIVE_CAMERA, MATEK
// -------------------------------------------------------


void send_telemetry(
    float sensor_readings[14],
    uint32_t state,
    char command_echo[60],
    uint8_t fsw_flags_arg,
    // GPS fields (read from PA1616D via Matek F405 UART)
    char gps_time[10],       // "HH:MM:SS"
    float gps_altitude,      // metres AMSL
    float gps_latitude,      // decimal degrees N
    float gps_longitude,     // decimal degrees W
    uint8_t gps_sats,
    // Matek F405 flight controller state string (e.g. "GLIDER", "MANUAL")
    char matek_state[20]
){

    char mode = ((fsw_flags_arg & 0x80) >= 1) ? 'S' : 'F'; // S = Simulation, F = Flight

        // --- Mechanism state string (ACTIVE_MECHS field) ---
    // Encode which servos/mechanisms are latched:
    // digit 1 = payload release servo (SG90)
    // digit 2 = port steering servo (TR-1160A)
    // digit 3 = starboard steering servo (TR-1160A)
    // digit 4 = egg release servo (SG90)
    // '1' = RELEASED/ACTIVE, '0' = LATCHED/INACTIVE
    char active_mechs[5];
    active_mechs[0] = ((fsw_flags_arg & 0x40) >= 1) ? '1' : '0'; // payload released
    active_mechs[1] = '0'; // port servo - expand when servo_state tracking added
    active_mechs[2] = '0'; // starboard servo
    active_mechs[3] = ((fsw_flags_arg & 0x20) >= 1) ? '1' : '0'; // egg released
    active_mechs[4] = '\0';




}

bool send_telemetry(float sensor_readings[21],uint32_t state, char command_echo[30], uint8_t fsw_flags_arg){
    
    char substate[15];
    //Serial.println("I'm try to send telemetry");
    time_t time = now(); 
    static int s_packet_count = 0;// static means that it doesn't reset when function goes out of scope i.e. it will keep its value until the MC resets. the s_ at the start is a nice reminder to programmers that it's static
    char sBuf[750]; // a buffer for the data telemetry 
    char mode;
    char ascii_state[13];
    char release_mech_state[10];
    
    
    
    // Send telemetry data over Serial
    Serial.print("Latitude: ");
    Serial.print(gps_data.latitude, 6);
    Serial.print(", Longitude: ");
    Serial.print(gps_data.longitude, 6);
    Serial.print(", Altitude: ");
    Serial.print(gps_data.altitude);
    Serial.print(" m, Speed: ");
    Serial.print(gps_data.speed);
    Serial.println(" m/s");
    
    return true; // telemetry sent successfully
}

