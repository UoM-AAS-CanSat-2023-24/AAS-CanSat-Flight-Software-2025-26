/*
 * CanSat 2026 — Team 1079
 * sensing.h  |  Local sensor drivers: INA260 + LM335AZ
 *
 * Sensors on this file:
 *   INA260   — I2C, voltage + current + power
 *   LM335AZ  — Analog pin A0, external temperature
 *
 * Everything else (barometer, IMU, GPS) comes from the F405 via MAVLink
 * and is handled in mavlink_handler.h
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "config.h"

// ============================================================================
//  SensorData struct — all sensor readings in one place.
//  MAVLink fields are filled by mavlink_handler.h
//  Local fields are filled by read_local_sensors() below
// ============================================================================
struct SensorData {
  // -- Barometer (SPL06 via F405 MAVLink) --
  float altitude_m    = 0.0f;   // AGL after CAL
  float pressure_kpa  = 0.0f;   // kPa

  // -- INA260 --
  float voltage_v    = 0.0f;
  float current_a    = 0.0f;
  float bus_power_w  = 0.0f;

  // -- LM335AZ external temperature --
  float ext_temp_c   = 0.0f;

  // -- ICM-20948 via ATTITUDE MAVLink (#30) — degrees/s --
  float gyro_r = 0.0f;
  float gyro_p = 0.0f;
  float gyro_y = 0.0f;

  // -- ICM-20948 via RAW_IMU MAVLink (#27) — raw counts --
  float accel_r = 0.0f;
  float accel_p = 0.0f;
  float accel_y = 0.0f;

  // -- GPS via GLOBAL_POSITION_INT MAVLink (#33) --
  uint8_t gps_hour  = 0;
  uint8_t gps_min   = 0;
  uint8_t gps_sec   = 0;
  float   gps_alt_m = 0.0f;   // AMSL metres
  float   gps_lat   = 0.0f;   // decimal degrees
  float   gps_lon   = 0.0f;
  uint8_t gps_sats  = 0;

  // -- SIM mode pressure override (set by SIMP command) --
  float sim_pressure_pa = 0.0f;
};

// ============================================================================
//  INA260 instance (file-scoped)
// ============================================================================
static Adafruit_INA260 ina260;

// ============================================================================
//  sensor_setup()
//  Called once in setup(). Returns false if a sensor doesn't respond so
//  main.cpp can retry.
// ============================================================================
bool sensor_setup() {
  if (!ina260.begin()) {
    Serial.println(F("[SENSOR] INA260 not found on I2C"));
    return false;
  }
  Serial.println(F("[SENSOR] INA260 OK"));
  // LM335AZ is passive analogue — no init needed
  return true;
}

// ============================================================================
//  read_local_sensors()
//  Called every loop iteration when not in PRE_LAUNCH or GROUNDED.
// ============================================================================
void read_local_sensors(SensorData& sd) {
  // --- INA260 ---------------------------------------------------------------
  // Library returns mV and mA so divide by 1000 to get V and A
  sd.voltage_v   = ina260.readBusVoltage() / 1000.0f;
  sd.current_a   = ina260.readCurrent()    / 1000.0f;
  sd.bus_power_w = ina260.readPower()      / 1000.0f;

  // --- LM335AZ --------------------------------------------------------------
  // LM335 outputs 10mV per Kelvin
  // Teensy 4.1 ADC: 10-bit (0-1023) at 3.3V reference
  // So: voltage = raw * (3.3 / 1023)
  //     temp_K  = voltage / 0.01
  //     temp_C  = temp_K - 273.15
  int   raw    = analogRead(PIN_LM335);
  float volts  = raw * (3.3f / 1023.0f);
  float temp_k = volts / 0.01f;
  sd.ext_temp_c = temp_k - 273.15f;
}

// ============================================================================
//  calibrate_ground()
//  Called when CAL command received on launch pad.
//  Latches current GPS AMSL altitude as the ground reference so all
//  subsequent altitude_m readings are AGL (above ground level).
// ============================================================================
void calibrate_ground(SensorData& sd, float& ground_alt_m) {
  ground_alt_m  = sd.gps_alt_m;
  sd.altitude_m = 0.0f;
  Serial.print(F("[CAL] Ground alt set to "));
  Serial.print(ground_alt_m, 1);
  Serial.println(F(" m AMSL"));
}