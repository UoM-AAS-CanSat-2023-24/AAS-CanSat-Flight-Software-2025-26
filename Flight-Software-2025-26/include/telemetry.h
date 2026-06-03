/*
 * CanSat 2026 — Team 1079
 * telemetry.cpp  |  Packet transmission and uplink command parsing
 *
 * send_telemetry_packet() — builds the full CSV packet and sends via XBee,
 *   then writes the same line to SD regardless of CX flag.
 *
 * parse_commands() — drains XBee RX buffer one line at a time and handles:
 *   CMD,1079,CX,ON|OFF
 *   CMD,1079,ST,hh:mm:ss|GPS
 *   CMD,1079,CAL
 *   CMD,1079,SIM,ENABLE|ACTIVATE|DISABLE
 *   CMD,1079,SIMP,<pressure_Pa>
 *   CMD,1079,MEC,ARM,ON        (custom: DISARMED → ARMED)
 *   CMD,1079,MEC,PAYLOAD,ON
 *   CMD,1079,MEC,PROBE,ON
 */

#pragma once
#include <RV-3028-C7.h> 

#include "mission_context.h"
#include "config.h"
#include "eeprom_store.h"
#include "sd_card.h"
#include "sensing.h"
#include "servo_control.h"
#include "camera_ctrl.h"
#include "mavlink_handler.h"

// ============================================================================
//  send_telemetry_packet
// ============================================================================
void send_telemetry_packet(MissionContext& ctx, RV3028& rtc) {
  // Mission time from RV3028
  char mission_time[12];
  snprintf(mission_time, sizeof(mission_time),
           "%02d:%02d:%02d", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  // GPS time (populated from MAVLink MSG #33)
  char gps_time[12];
  snprintf(gps_time, sizeof(gps_time),
           "%02d:%02d:%02d",
           ctx.sd.gps_hour, ctx.sd.gps_min, ctx.sd.gps_sec);

  // ACTIVE_MECHS bitmask
  uint8_t active_mechs = 0;
  if (ctx.flags & FLAG_PAYLOAD_RELEASED) active_mechs |= 0x01;
  if (ctx.flags & FLAG_PROBE_RELEASED)   active_mechs |= 0x02;

  char buf[384];
  snprintf(buf, sizeof(buf),
    "%s,"          // TEAM_ID
    "%s,"          // MISSION_TIME
    "%lu,"         // PACKET_COUNT
    "%c,"          // MODE  F/S
    "%s,"          // STATE
    "%.1f,"        // ALTITUDE  m AGL
    "%.1f,"        // TEMPERATURE  °C  (LM335AZ)
    "%.2f,"        // PRESSURE  kPa
    "%.2f,"        // VOLTAGE  V
    "%.3f,"        // CURRENT  A
    "%.2f,%.2f,%.2f,"     // GYRO_R,P,Y  °/s
    "%.2f,%.2f,%.2f,"     // ACCEL_R,P,Y  raw counts
    "%s,"          // GPS_TIME
    "%.1f,"        // GPS_ALTITUDE  m AMSL
    "%.4f,"        // GPS_LATITUDE
    "%.4f,"        // GPS_LONGITUDE
    "%u,"          // GPS_SATS
    "%s,"          // CMD_ECHO
    "%s,"          // SUBSTATE  (mirrors STATE until sub-states are added)
    "%.1f,"        // MAIN_SOC  % (placeholder)
    "%.2f,"        // BUS_POWER  W
    "%u,"          // ACTIVE_MECHS  bitmask
    "%u,"          // ACTIVE_CAMERA  bitmask
    "%u\r",        // MATEK  1=heartbeat OK
    TEAM_ID_STR,
    mission_time,
    (unsigned long)ctx.packet_count,
    ctx.mode_char(),
    state_name(ctx.state),
    ctx.sd.altitude_m,
    ctx.sd.ext_temp_c,
    ctx.sd.pressure_kpa,
    ctx.sd.voltage_v,
    ctx.sd.current_a,
    ctx.sd.gyro_r, ctx.sd.gyro_p, ctx.sd.gyro_y,
    ctx.sd.accel_r, ctx.sd.accel_p, ctx.sd.accel_y,
    gps_time,
    ctx.sd.gps_alt_m,
    ctx.sd.gps_lat,
    ctx.sd.gps_lon,
    ctx.sd.gps_sats,
    ctx.cmd_echo,
    state_name(ctx.state),
    0.0f,                       // MAIN_SOC placeholder
    ctx.sd.bus_power_w,
    active_mechs,
    camera_active_flags(),
    matek_heartbeat_ok()
  );

  if (ctx.cx_on()) {
    XBEE_SERIAL.print(buf);
  }

  sd_write_line(buf);
  ctx.packet_count++;
}

// ============================================================================
//  parse_commands
// ============================================================================
static char rx_buf[128];
static int  rx_idx = 0;

void parse_commands(MissionContext& ctx, RV3028& rtc) {
  while (XBEE_SERIAL.available()) {
    char c = (char)XBEE_SERIAL.read();

    if (c == '\r' || c == '\n') {
      if (rx_idx == 0) continue;
      rx_buf[rx_idx] = '\0';
      rx_idx = 0;

      char* tok = strtok(rx_buf, ",");
      if (!tok || strcmp(tok, "CMD") != 0)  goto done;

      tok = strtok(nullptr, ",");           // TEAM_ID
      if (!tok || atoi(tok) != TEAM_ID_INT) goto done;

      tok = strtok(nullptr, ",");           // command keyword
      if (!tok) goto done;

      // CX ------------------------------------------------------------------
      if (strcmp(tok, "CX") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "ON") == 0) {
          ctx.flags |= FLAG_CX_ON;
          ctx.packet_count = 0;
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "CXON",  31);
        } else {
          ctx.flags &= ~FLAG_CX_ON;
          strncpy(ctx.cmd_echo, "CXOFF", 31);
        }
      }

      // ST ------------------------------------------------------------------
      else if (strcmp(tok, "ST") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "GPS") == 0) {
          rtc.setTime(ctx.sd.gps_sec, ctx.sd.gps_min, ctx.sd.gps_hour,
                      1, 1, 1, 25);
        } else {
          int hh = 0, mm = 0, ss = 0;
          if (sscanf(p, "%d:%d:%d", &hh, &mm, &ss) == 3) {
            rtc.setTime((uint8_t)ss, (uint8_t)mm, (uint8_t)hh,
                        1, 1, 1, 25);
          }
        }
        strncpy(ctx.cmd_echo, "ST", 31);
      }

      // CAL -----------------------------------------------------------------
      else if (strcmp(tok, "CAL") == 0) {
        calibrate_ground(ctx.sd, ctx.ground_alt_m);
        eeprom_save(ctx);
        strncpy(ctx.cmd_echo, "CAL", 31);
      }

      // SIM -----------------------------------------------------------------
      else if (strcmp(tok, "SIM") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "ENABLE") == 0) {
          ctx.flags |=  FLAG_SIM_ENABLED;
          ctx.flags &= ~FLAG_SIM_ACTIVE;
          strncpy(ctx.cmd_echo, "SIMEN",  31);
        } else if (strcmp(p, "ACTIVATE") == 0) {
          if (ctx.flags & FLAG_SIM_ENABLED) {
            ctx.flags |= FLAG_SIM_ACTIVE;
            strncpy(ctx.cmd_echo, "SIMACT", 31);
          }
        } else if (strcmp(p, "DISABLE") == 0) {
          ctx.flags &= ~(FLAG_SIM_ENABLED | FLAG_SIM_ACTIVE);
          strncpy(ctx.cmd_echo, "SIMDIS", 31);
        }
        eeprom_save(ctx);
      }

      // SIMP ----------------------------------------------------------------
      else if (strcmp(tok, "SIMP") == 0) {
        char* p = strtok(nullptr, ",");
        if (p && (ctx.flags & FLAG_SIM_ACTIVE)) {
          ctx.sd.sim_pressure_pa = (float)atol(p);
          strncpy(ctx.cmd_echo, "SIMP", 31);
        }
      }

      // MEC -----------------------------------------------------------------
      else if (strcmp(tok, "MEC") == 0) {
        char* device = strtok(nullptr, ",");
        char* onoff  = strtok(nullptr, ",");
        if (!device || !onoff) goto done;

        if (strcmp(device, "ARM") == 0 && strcmp(onoff, "ON") == 0) {
          if (ctx.state == MissionState::LAUNCH_PAD_DISARMED) {
            ctx.state = MissionState::LAUNCH_PAD_ARMED;
            eeprom_save(ctx);
          }
          strncpy(ctx.cmd_echo, "MECARM", 31);

        } else if (strcmp(device, "PAYLOAD") == 0 && strcmp(onoff, "ON") == 0) {
          payload_release_actuate();
          ctx.flags |= FLAG_PAYLOAD_RELEASED;
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "MECPAY", 31);

        } else if (strcmp(device, "PROBE") == 0 && strcmp(onoff, "ON") == 0) {
          egg_release_actuate();
          ctx.flags |= FLAG_PROBE_RELEASED;
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "MECPRB", 31);
        }
      }

      done:;

    } else {
      if (rx_idx < (int)sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = c;
      }
    }
  }
}
