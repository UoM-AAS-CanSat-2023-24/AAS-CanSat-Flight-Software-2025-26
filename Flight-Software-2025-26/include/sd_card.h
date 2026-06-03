/*
 * CanSat 2026 — Team 1079
 * sd_card.h  |  SD card logging via Teensy 4.1 built-in SDIO
 *
 * Writes the same CSV line as the transmitted telemetry packet.
 * Always writes regardless of CX flag — onboard record is always complete.
 * flush() called every write to protect against mid-flight resets.
 *
 * File: Flight_1079.csv
 */

#pragma once
#include <Arduino.h>
#include <SD.h>

static File log_file;
static bool sd_ok = false;

// ============================================================================
void sd_setup() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("[SD] Init failed — onboard logging disabled"));
    sd_ok = false;
    return;
  }
  // FILE_WRITE appends if file exists, creates if not
  log_file = SD.open("Flight_1079.csv", FILE_WRITE);
  if (!log_file) {
    Serial.println(F("[SD] Cannot open Flight_1079.csv"));
    sd_ok = false;
    return;
  }
  sd_ok = true;
  Serial.println(F("[SD] Ready: Flight_1079.csv"));
}

// ============================================================================
void sd_write_header(const char* header) {
  if (!sd_ok) return;
  // Only write header if file is brand new (size 0)
  if (log_file.size() == 0) {
    log_file.println(header);
    log_file.flush();
  }
}

// ============================================================================
void sd_write_line(const char* line) {
  if (!sd_ok) return;
  log_file.println(line);
  log_file.flush();  // flush every write — guards against reset data loss
}