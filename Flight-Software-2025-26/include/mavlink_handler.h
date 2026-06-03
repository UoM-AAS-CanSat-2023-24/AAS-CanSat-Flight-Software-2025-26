/*
 * CanSat 2026 — Team 1079
 * mavlink_handler.cpp  |  MAVLink polling and SIM altitude override
 *
 * Consumes three MAVLink messages from the Matek F405 on Serial1:
 *   #33  GLOBAL_POSITION_INT  lat/lon (/1e7 → °)  alt (/1000 → m)
 *   #27  RAW_IMU              xacc, yacc, zacc (raw counts)
 *   #30  ATTITUDE             roll/pitch/yaw (rad → ° via × 180/PI)
 */

#pragma once
#include "mission_context.h"
#include "config.h"
#include <ardupilotmega/mavlink.h>


static mavlink_message_t mav_msg;
static mavlink_status_t  mav_status;

static uint32_t last_mav_rx_ms    = 0;
static uint32_t prev_mav_rx_count = 0;

// ============================================================================
void poll_mavlink(MissionContext& ctx) {
  while (MAVLINK_SERIAL.available()) {
    uint8_t b = (uint8_t)MAVLINK_SERIAL.read();
    if (!mavlink_parse_char(MAVLINK_COMM_0, b, &mav_msg, &mav_status)) continue;

    // Update heartbeat tracker
    if (mav_status.packet_rx_success_count != prev_mav_rx_count) {
      prev_mav_rx_count = mav_status.packet_rx_success_count;
      last_mav_rx_ms    = millis();
    }

    switch (mav_msg.msgid) {

      // MSG #33 — GLOBAL_POSITION_INT ---------------------------------------
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gpi;
        mavlink_msg_global_position_int_decode(&mav_msg, &gpi);
        ctx.sd.gps_lat   = (float)gpi.lat / 10000000.0f;
        ctx.sd.gps_lon   = (float)gpi.lon / 10000000.0f;
        ctx.sd.gps_alt_m = (float)gpi.alt / 1000.0f;
        // Derive AGL altitude from GPS AMSL minus ground reference
        // (overwritten by barometric in flight; useful as sanity check)
        ctx.sd.altitude_m = ctx.sd.gps_alt_m - ctx.ground_alt_m;
        break;
      }

      // MSG #27 — RAW_IMU ---------------------------------------------------
      case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t raw;
        mavlink_msg_raw_imu_decode(&mav_msg, &raw);
        ctx.sd.accel_r = (float)raw.xacc;
        ctx.sd.accel_p = (float)raw.yacc;
        ctx.sd.accel_y = (float)raw.zacc;
        break;
      }

      // MSG #30 — ATTITUDE --------------------------------------------------
      case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&mav_msg, &att);
        ctx.sd.gyro_r = att.roll  * (180.0f / PI);
        ctx.sd.gyro_p = att.pitch * (180.0f / PI);
        ctx.sd.gyro_y = att.yaw   * (180.0f / PI);
        break;
      }

      default: break;
    }
  }
}

// ============================================================================
//  SIM mode altitude override
//  Hypsometric formula: h_amsl = 44330 × (1 − (P/P₀)^0.1903)
// ============================================================================
void update_altitude_from_sim(MissionContext& ctx) {
  if (!ctx.sim_active()) return;
  float amsl          = 44330.0f * (1.0f - powf(ctx.sd.sim_pressure_pa / 101325.0f, 0.1903f));
  ctx.sd.altitude_m   = amsl - ctx.ground_alt_m;
  ctx.sd.pressure_kpa = ctx.sd.sim_pressure_pa / 1000.0f;
}

// ============================================================================
uint8_t matek_heartbeat_ok() {
  return ((millis() - last_mav_rx_ms) < 2000u) ? 1 : 0;
}
