/*
 * CanSat 2026 — Team 1079
 * eeprom_store.h
 */

#pragma once
#include "mission_context.h"

void eeprom_save(const MissionContext& ctx); // save the entire context to EEPROM
void eeprom_restore(MissionContext& ctx); // restore context from EEPROM (called at boot) — handles processor resets during flight, as well as restoring packet count for telemetry continuity
