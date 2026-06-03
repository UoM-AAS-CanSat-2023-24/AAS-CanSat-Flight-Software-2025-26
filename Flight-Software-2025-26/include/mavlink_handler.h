/*
 * CanSat 2026 — Team 1079
 * mavlink_handler.h  |  MAVLink polling and SIM altitude override
 */

#pragma once
#include "mission_context.h"

void poll_mavlink(MissionContext& ctx);
void update_altitude_from_sim(MissionContext& ctx);
uint8_t matek_heartbeat_ok();   // returns 1 if MAVLink received within last 2 s
