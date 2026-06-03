/*
 * CanSat 2026 — Team 1079
 * servo_control.h  |  Payload release and egg release servos
 *
 * Hardware:
 *   Payload release : standard servo on PIN_SERVO_PAYLOAD
 *   Egg release     : Emax ES3053 on PIN_SERVO_EGG (more torque than MG90)
 *
 * Paraglider steering is handled entirely by the F405 flight controller —
 * no steering servos are controlled from the Teensy.
 *
 * IMPORTANT: Calibrate PAYLOAD_RELEASE_DEG and EGG_RELEASE_DEG on the bench
 * before flight — verify the servo rotates in the correct direction and
 * actually releases the mechanism.
 */

#pragma once
#include <Arduino.h>
#include <PWMServo.h>
#include "config.h"

// Servo positions in degrees — CALIBRATE ON BENCH BEFORE FLIGHT
#define PAYLOAD_LOCKED_DEG    90
#define PAYLOAD_RELEASE_DEG    0   // direction/amount to release — verify on bench

#define EGG_LOCKED_DEG        90
#define EGG_RELEASE_DEG        0   // direction/amount to release — verify on bench

static PWMServo servo_payload;
static PWMServo servo_egg;

// ============================================================================
void payload_release_servo_setup(int pin) {
  servo_payload.attach(pin);
  servo_payload.write(PAYLOAD_LOCKED_DEG);
  Serial.println(F("[SERVO] Payload servo initialised (locked)"));
}

void egg_release_servo_setup(int pin) {
  servo_egg.attach(pin);
  servo_egg.write(EGG_LOCKED_DEG);
  Serial.println(F("[SERVO] Egg servo initialised (locked)"));
}

// ============================================================================
void payload_release_actuate() {
  servo_payload.write(PAYLOAD_RELEASE_DEG);
  Serial.println(F("[SERVO] Payload servo actuated"));
}

void egg_release_actuate() {
  servo_egg.write(EGG_RELEASE_DEG);
  Serial.println(F("[SERVO] Egg servo actuated"));
}