/*
 * CanSat 2026 — Team 1079
 * telemetry.h  |  Packet transmission and command parsing
 */

#pragma once
#include "mission_context.h"
#include <RV-3028-C7.h>

void send_telemetry_packet(MissionContext& ctx, RV3028& rtc);
void parse_commands(MissionContext& ctx, RV3028& rtc);
