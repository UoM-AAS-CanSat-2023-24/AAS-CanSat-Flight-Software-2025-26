/*
 * CanSat 2026 — Team 1079
 * camera_ctrl.h  |  Camera control via digital pins
 *
 * Each Seeed XIAO ESP32S3 is triggered by a digital pin:
 *   HIGH = record
 *   LOW  = stop
 *
 * Pin assignments are in config.h
 */

#pragma once
#include <Arduino.h>
#include "config.h"

static bool cam_release_active = false;
static bool cam_ground_active  = false;

void camera_setup() {
  pinMode(PIN_CAM_RELEASE, OUTPUT);
  pinMode(PIN_CAM_GROUND,  OUTPUT);
  digitalWrite(PIN_CAM_RELEASE, LOW);
  digitalWrite(PIN_CAM_GROUND,  LOW);
  Serial.println(F("[CAM] Camera pins initialised"));
}

void camera_release_start() { digitalWrite(PIN_CAM_RELEASE, HIGH); cam_release_active = true;  }
void camera_release_stop()  { digitalWrite(PIN_CAM_RELEASE, LOW);  cam_release_active = false; }
void camera_ground_start()  { digitalWrite(PIN_CAM_GROUND,  HIGH); cam_ground_active  = true;  }
void camera_ground_stop()   { digitalWrite(PIN_CAM_GROUND,  LOW);  cam_ground_active  = false; }

uint8_t camera_active_flags() {
  uint8_t flags = 0;
  if (cam_release_active) flags |= 0x01;
  if (cam_ground_active)  flags |= 0x02;
  return flags;
}