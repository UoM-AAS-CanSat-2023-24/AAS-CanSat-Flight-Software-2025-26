/*
 * CanSat 2026 — Team 1079 — Manchester Satellite Development Group
 * main.cpp  |  Flight Software
 *
 * MCU      : Teensy 4.1
 * Co-proc  : Matek F405-miniTE  →  Serial1 (RX=1, TX=1) @ 115200 baud, MAVLink
 * RTC      : RV3028              →  I2C (Wire)
 * Radio    : XBee 3 Pro          →  Serial6 (RX=6, TX=6) @ 9600 baud
 * SD       : Teensy 4.1 SDIO     →  BUILTIN_SDCARD
 *
 * MAVLink messages consumed:
 *   #33  GLOBAL_POSITION_INT  lat/lon (/1e7 → °)  alt (/1000 → m)
 *   #27  RAW_IMU              xacc, yacc, zacc (raw counts)
 *   #30  ATTITUDE             roll/pitch/yaw (rad → ° via × 180/PI)
 *
 * Telemetry packet (1 Hz CSV, '\r' terminated):
 *   TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE,
 *   ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT,
 *   GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
 *   GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
 *   CMD_ECHO, SUBSTATE, MAIN_SOC, BUS_POWER, ACTIVE_MECHS,
 *   ACTIVE_CAMERA, MATEK
 *
 * FSW loop structure (mirrors CDR flowchart):
 *   1. Check for GCS commands (always)
 *   2. If NOT (PRE_LAUNCH or GROUNDED): poll sensors
 *   3. If telemetry active AND timer elapsed: send telemetry
 *   4. State machine logic
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RV-3028-C7.h>
 
#include "config.h" // compile-time constants
#include "mission_context.h" // MissionContext struct, MissionState enum, state_name()
#include "sensing.h" // SensorData struct, sensor_setup(), read_local_sensors()
#include "mavlink_handler.h"
#include "flight_logic.h"
#include "telemetry.h"
#include "eeprom_store.h"
#include "servo_control.h"

#include "camera_ctrl.h"
#include "sd_card.h"

//  Globals — one context object and the RTC. That's it.
MissionContext ctx;
RV3028         rtc;

//  Timing
uint32_t last_telem_ms  = 0;
uint32_t last_buzzer_ms = 0;
uint32_t last_eeprom_ms = 0;

//  Helper: change state, log it, save to EEPROM
void set_state(MissionState new_state) {
  if (ctx.state == new_state) return;
  ctx.state = new_state;
  eeprom_save(ctx);
  Serial.print(F("[STATE] -> "));
  Serial.println(state_name(new_state));
}

//  Inline helpers (keep state checks readable in the switch)
inline bool is_pre_launch() {
  return ctx.state == MissionState::LAUNCH_PAD_DISARMED ||
         ctx.state == MissionState::LAUNCH_PAD_ARMED;
}
inline bool is_descent(MissionState s) {
  return s == MissionState::DESCENT_PRE_PAYLOAD_RELEASE ||
         s == MissionState::DESCENT_PAYLOAD_RELEASE     ||
         s == MissionState::DESCENT_PRE_PROBE_RELEASE   ||
         s == MissionState::DESCENT_PROBE_RELEASE       ||
         s == MissionState::DESCENT_POST_PROBE_RELEASE;
}


void setup() {
 Serial.begin(115200);   // USB debug — comment out before flight
 delay(400);
 Serial.println(F("[FSW] Booting..."));

  Wire.begin();

  // RV3028
  if (!rtc.begin()) {
    Serial.println(F("[RTC] RV3028 not found — awaiting ST command"));
  } else {
    rtc.set24HourMode();
    Serial.println(F("[RTC] RV3028 OK"));
  }

  // Restore state from EEPROM (handles processor resets)
  eeprom_restore(ctx);

  // Serials
  MAVLINK_SERIAL.begin(MAVLINK_BAUD);
  XBEE_SERIAL.begin(XBEE_BAUD);

  // Sensors
  while (!sensor_setup()) {
    Serial.println(F("[SENSOR] Init failed, retrying..."));
    delay(500);
  }

  // Servos
  payload_release_servo_setup(PIN_SERVO_PAYLOAD);
  egg_release_servo_setup(PIN_SERVO_EGG);

  // Cameras
  camera_setup();

  // SD
  sd_setup();
  sd_write_header(
    "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,"
    "ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,CURRENT,"
    "GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,"
    "GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,"
    "CMD_ECHO,SUBSTATE,MAIN_SOC,BUS_POWER,ACTIVE_MECHS,ACTIVE_CAMERA,MATEK"
  );

  Serial.print(F("[FSW] Ready. State: "));
  Serial.println(state_name(ctx.state));
}

void loop() {
  uint32_t now = millis();

  // 1. GCS commands — checked every iteration 
  parse_commands(ctx, rtc);

  // 2. Sensor polling — skipped in PRE_LAUNCH and GROUNDED 
  if (!is_pre_launch() && ctx.state != MissionState::GROUNDED) {
    poll_mavlink(ctx);
    read_local_sensors(ctx.sd);
    update_altitude_from_sim(ctx);
  }

  // 3. Telemetry — 1 Hz 
  if (now - last_telem_ms >= TELEM_INTERVAL_MS) {
    send_telemetry_packet(ctx, rtc);
    last_telem_ms = now;
  }

  // 4. Periodic EEPROM flush 
  if (now - last_eeprom_ms >= EEPROM_SAVE_MS) {
    eeprom_save(ctx);
    last_eeprom_ms = now;
  }

  //  State machine
  switch (ctx.state) {

    case MissionState::LAUNCH_PAD_DISARMED:
      // Waiting for CMD,1079,MEC,ARM,ON from GCS — handled in parse_commands()
      break;

    case MissionState::LAUNCH_PAD_ARMED:
      if (launch_detected(ctx)) {

        // We have the power for it so we're just gonna keep both cameras on for the whole flight — no need to swap at apogee 
        camera_release_start();
        camera_ground_start();
        set_state(MissionState::ASCENT);
      }
      break;

    case MissionState::ASCENT:
      // Track apogee
      if (ctx.sd.altitude_m > ctx.apogee_m) { //check if reached apogee
        ctx.apogee_m   = ctx.sd.altitude_m;
        ctx.desc_count = 0;
        eeprom_save(ctx);
      } else if (ctx.sd.altitude_m < ctx.apogee_m) { // check if descending after apogee
        ctx.desc_count++;
      }

      // Grounded check — covers aborted / very short flights
      if (grounded_detected(ctx)) {
        set_state(MissionState::GROUNDED);
        break;
      }

      if (ctx.desc_count >= APOGEE_CONFIRM_COUNT) {
        //camera_release_stop();
        //camera_ground_start();
        set_state(MissionState::APOGEE);
      }
      break;

    case MissionState::APOGEE:
      // One-shot transition — camera swap already done above
      set_state(MissionState::DESCENT_PRE_PAYLOAD_RELEASE);
      break;

    case MissionState::DESCENT_PRE_PAYLOAD_RELEASE:
      if (grounded_detected(ctx)) { set_state(MissionState::GROUNDED); break; }


      // check if reached payload release altitude (fraction of apogee) — also check apogee > 0 to avoid false trigger on descent from launch pad
      if ((ctx.apogee_m > 0.0f) &&
          (ctx.sd.altitude_m <= PAYLOAD_RELEASE_FRAC * ctx.apogee_m) &&
          !(ctx.flags & FLAG_PAYLOAD_RELEASED)) { 
        set_state(MissionState::DESCENT_PAYLOAD_RELEASE);
      }
      break;

      case MissionState::DESCENT_PAYLOAD_RELEASE:
      if (!(ctx.flags & FLAG_PAYLOAD_RELEASED)) { // one-shot payload release actuation
        payload_release_actuate();
        ctx.flags |= FLAG_PAYLOAD_RELEASED; // set flag to prevent re-actuation,
        eeprom_save(ctx);
        Serial.println(F("[MECH] Payload released"));
      }
      set_state(MissionState::DESCENT_PRE_PROBE_RELEASE);
      break;

    case MissionState::DESCENT_PRE_PROBE_RELEASE:

      if (grounded_detected(ctx)) { set_state(MissionState::GROUNDED); break; }

      if (ctx.sd.altitude_m <= EGG_RELEASE_ALT_M) {
        set_state(MissionState::DESCENT_PROBE_RELEASE);
      }
      break;

    case MissionState::DESCENT_PROBE_RELEASE:
      if (!(ctx.flags & FLAG_PROBE_RELEASED)) {
        egg_release_actuate();
        ctx.flags |= FLAG_PROBE_RELEASED;
        eeprom_save(ctx);
        Serial.println(F("[MECH] Egg released"));
      }
      set_state(MissionState::DESCENT_POST_PROBE_RELEASE);
      break;

    case MissionState::DESCENT_POST_PROBE_RELEASE:
      if (grounded_detected(ctx)) {
        set_state(MissionState::GROUNDED);
      }
      break;

    case MissionState::GROUNDED:
      // No autonomous transitions — listen for commands only
      break;

    case MissionState::FAULT:
      // Keep transmitting for diagnosis, no actuations
      break;

    default: 
      // Flowchart: CRITICAL ERROR — revert to ASCENT
      Serial.println(F("[FSW] CRITICAL ERROR: unknown state — reverting to ASCENT"));
      ctx.desc_count = 0;
      set_state(MissionState::ASCENT);
      break;
  }
}


