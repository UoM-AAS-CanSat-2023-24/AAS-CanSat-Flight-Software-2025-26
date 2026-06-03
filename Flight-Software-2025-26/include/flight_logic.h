/*
 * CanSat 2026 — Team 1079
 * flight_logic.h  |  Launch and grounded detection
 */

#pragma once
#include "mission_context.h"

bool launch_detected(const MissionContext& ctx);
bool grounded_detected(MissionContext& ctx);   // non-const: owns the debounce counter
