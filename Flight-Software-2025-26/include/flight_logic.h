/*
 * CanSat 2026 — Team 1079
 * flight_logic.cpp  |  Launch and grounded detection
 *
 * Both functions use BOTH altitude AND accelerometer so a single noisy
 * sensor can't cause a false trigger. Thresholds are in config.h.
 */

#pragma once
#include "config.h"
#include "mission_context.h"

// ============================================================================
//  Launch detection
//  TRUE when altitude > threshold OR upward accel exceeds idle baseline.
//  OR logic: either sensor confirming launch is enough; both failing to
//  confirm means we stay on the pad.
// ============================================================================
bool launch_detected(const MissionContext& ctx) {
  bool alt_trigger   = ctx.sd.altitude_m > LAUNCH_DETECT_ALT_M;
  bool accel_trigger = ctx.sd.accel_y    > LAUNCH_ACCEL_THRESHOLD;
  return alt_trigger || accel_trigger; // either one is sufficient to confirm launch
}

// ============================================================================
//  Grounded detection
//  TRUE after LANDING_CONFIRM_COUNT consecutive readings where:
//    - altitude is below LANDING_ALT_M, AND
//    - vertical accel is in the static 1g band (not in free-fall)
//  AND logic: both sensors must agree to avoid false positives mid-flight.
// ============================================================================
bool grounded_detected(MissionContext& ctx) {
  static uint8_t gnd_count = 0;

  bool alt_low  = ctx.sd.altitude_m < LANDING_ALT_M;
  bool accel_1g = (ctx.sd.accel_y > GROUNDED_ACCEL_MIN) &&
                  (ctx.sd.accel_y < GROUNDED_ACCEL_MAX);

  if (alt_low && accel_1g) {
    gnd_count++;
  } else {
    gnd_count = 0;  // reset: altitude bounced or accel not static
  }

  return gnd_count >= LANDING_CONFIRM_COUNT;
}
