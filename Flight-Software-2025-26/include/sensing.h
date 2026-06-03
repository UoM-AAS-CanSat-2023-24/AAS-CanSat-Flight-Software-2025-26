/*
 * CanSat 2026 — Team 1079
 * sensing.h  |  SensorData struct + public API
 */

#pragma once
#include <Arduino.h>

// All sensor readings in one place. Populated by sensing.cpp, read everywhere.
struct SensorData {
  // Barometer (SPL06-001 via Matek F405 MAVLink)
  float altitude_m    = 0.0f;   // AGL after CAL
  float temperature_c = 0.0f;   // SPL06 internal temp (°C)
  float pressure_kpa  = 0.0f;   // kPa

  // INA260 power monitor
  float voltage_v    = 0.0f;
  float current_a    = 0.0f;
  float bus_power_w  = 0.0f;

  // ICM-20948 via ATTITUDE MAVLink (#30)  — degrees/s
  float gyro_r = 0.0f;
  float gyro_p = 0.0f;
  float gyro_y = 0.0f;

  // ICM-20948 via RAW_IMU MAVLink (#27)  — raw counts
  float accel_r = 0.0f;
  float accel_p = 0.0f;
  float accel_y = 0.0f;

  // GPS via GLOBAL_POSITION_INT MAVLink (#33)
  uint8_t gps_hour  = 0;
  uint8_t gps_min   = 0;
  uint8_t gps_sec   = 0;
  float   gps_alt_m = 0.0f;   // AMSL metres
  float   gps_lat   = 0.0f;   // decimal degrees
  float   gps_lon   = 0.0f;
  uint8_t gps_sats  = 0;

  // LM335AZ external temperature (analogue, Teensy A0)
  float ext_temp_c = 0.0f;

  // SIM mode — set by SIMP command in telemetry.cpp
  float sim_pressure_pa = 0.0f;
};

// sensing.cpp public API
bool sensor_setup();
void read_local_sensors(SensorData& sd);
void calibrate_ground(SensorData& sd, float& ground_alt_m);
