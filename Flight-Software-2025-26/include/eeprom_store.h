/*
 * CanSat 2026 — Team 1079
 * eeprom_store.cpp  |  EEPROM layout and read/write
 *
 * Layout (Teensy 4.1, 4284 bytes emulated EEPROM):
 *   Addr  0      magic byte  0xCA
 *   Addr  1      MissionState (uint8_t)
 *   Addr  2-5    apogee_m    (float)
 *   Addr  6-9    ground_alt_m (float)
 *   Addr 10-13   packet_count (uint32_t)
 *   Addr 14      flags — SIM bits only
 */

#pragma once
#include <EEPROM.h>
#include "mission_context.h"

#define MAGIC_VAL   0xCA
#define ADDR_MAGIC   0
#define ADDR_STATE   1
#define ADDR_APOG    2
#define ADDR_GNDA    6
#define ADDR_PKTC   10
#define ADDR_FLAGS  14

// Flag bits in ADDR_FLAGS
void eeprom_save(const MissionContext& ctx) {
  EEPROM.update(ADDR_MAGIC, MAGIC_VAL);
  EEPROM.update(ADDR_STATE, (uint8_t)ctx.state);
  EEPROM.put(ADDR_APOG, ctx.apogee_m);
  EEPROM.put(ADDR_GNDA, ctx.ground_alt_m);
  EEPROM.put(ADDR_PKTC, ctx.packet_count);
  EEPROM.update(ADDR_FLAGS, ctx.flags & (FLAG_SIM_ENABLED | FLAG_SIM_ACTIVE));
}

void eeprom_restore(MissionContext& ctx) {
  if (EEPROM.read(ADDR_MAGIC) != MAGIC_VAL) {
    Serial.println(F("[EEPROM] Fresh — using defaults"));
    return;
  }

  uint8_t raw = EEPROM.read(ADDR_STATE);
  ctx.state = (raw <= (uint8_t)MissionState::GROUNDED)
                ? (MissionState)raw
                : MissionState::ASCENT;   // safe fallback

  EEPROM.get(ADDR_APOG, ctx.apogee_m);
  EEPROM.get(ADDR_GNDA, ctx.ground_alt_m);
  EEPROM.get(ADDR_PKTC, ctx.packet_count);
  uint8_t sf = EEPROM.read(ADDR_FLAGS);
  ctx.flags |= (sf & (FLAG_SIM_ENABLED | FLAG_SIM_ACTIVE));

  Serial.print(F("[EEPROM] Restored state="));
  Serial.print(state_name(ctx.state));
  Serial.print(F("  apogee="));
  Serial.print(ctx.apogee_m, 1);
  Serial.print(F(" m  pkt="));
  Serial.println(ctx.packet_count);
}
