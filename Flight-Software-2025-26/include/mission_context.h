/*
 * CanSat 2026 — Team 1079
 * mission_context.h  |  Shared mission state passed by reference to all modules
 *
 * Every library that needs to read or write mission state takes a
 * MissionContext& parameter. No extern globals, no hidden dependencies.
 */

#pragma once
#include <Arduino.h>
#include "sensing.h"  // SensorData

// ============================================================================
//  State machine
// ============================================================================
enum class MissionState : uint8_t {
  LAUNCH_PAD_DISARMED         = 0,
  LAUNCH_PAD_ARMED            = 1,
  ASCENT                      = 2,
  APOGEE                      = 3,
  DESCENT_PRE_PAYLOAD_RELEASE = 4,
  DESCENT_PAYLOAD_RELEASE     = 5,
  DESCENT_PRE_PROBE_RELEASE   = 6,
  DESCENT_PROBE_RELEASE       = 7,
  DESCENT_POST_PROBE_RELEASE  = 8,
  GROUNDED                    = 9,
  FAULT                       = 255
};

inline const char* state_name(MissionState s) { // Returns the ASCII string transmitted in telemetry STATE field
  switch (s) {
    case MissionState::LAUNCH_PAD_DISARMED:         return "LAUNCH_PAD_DISARMED";
    case MissionState::LAUNCH_PAD_ARMED:            return "LAUNCH_PAD_ARMED";
    case MissionState::ASCENT:                      return "ASCENT";
    case MissionState::APOGEE:                      return "APOGEE";
    case MissionState::DESCENT_PRE_PAYLOAD_RELEASE: return "DESC_PRE_PAYLOAD";
    case MissionState::DESCENT_PAYLOAD_RELEASE:     return "DESC_PAYLOAD_REL";
    case MissionState::DESCENT_PRE_PROBE_RELEASE:   return "DESC_PRE_PROBE";
    case MissionState::DESCENT_PROBE_RELEASE:       return "DESC_PROBE_REL";
    case MissionState::DESCENT_POST_PROBE_RELEASE:  return "DESC_POST_PROBE";
    case MissionState::GROUNDED:                    return "GROUNDED";
    case MissionState::FAULT:                       return "FAULT";
    default:                                        return "UNKNOWN";
  }
}

// ============================================================================
//  FSW flags (bit 7-0, MSB to LSB) — all persisted in EEPROM and transmitted in telemetry
// Bit 7 = CX (telemetry TX) ON/OFF
// Bit 6 = SIM mode ENABLED (set by GCS command, persists across power cycles, allows ACTIVATE)
// Bit 5 = SIM mode ACTIVE (set by GCS command, overrides sensor altitude with SIMP value)
// Bit 4 = PAYLOAD released
// Bit 3 = PROBE (egg) released
// Bit 2-0 = reserved for future use
// ============================================================================
#define FLAG_CX_ON            (1u << 7)
#define FLAG_SIM_ENABLED      (1u << 6)
#define FLAG_SIM_ACTIVE       (1u << 5)
#define FLAG_PAYLOAD_RELEASED (1u << 4)
#define FLAG_PROBE_RELEASED   (1u << 3)

// ============================================================================
//  Shared mission context struct
// ============================================================================
struct MissionContext {
  MissionState state    = MissionState::LAUNCH_PAD_DISARMED;
  uint8_t      flags    = 0x00;

  float    ground_alt_m = 0.0f;  // AMSL altitude of launch pad (set at CAL)
  float    apogee_m     = 0.0f;  // highest AGL altitude recorded
  int      desc_count   = 0;     // consecutive readings below apogee
  uint32_t packet_count = 0;     // TX counter, persisted in EEPROM
  char     cmd_echo[32] = "-";   // last processed command

  SensorData sd;                 // all sensor readings

  // Convenience flag accessors
  inline bool cx_on()      const { return flags & FLAG_CX_ON; }
  inline bool sim_active() const { return flags & FLAG_SIM_ACTIVE; }
  inline char mode_char()  const { return sim_active() ? 'S' : 'F'; } // 'S' for SIM mode, 'F' for real flight
};
