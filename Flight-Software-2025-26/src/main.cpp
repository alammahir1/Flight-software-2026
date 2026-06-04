/*
 * CanSat 2026 — Team 1079 — Manchester Satellite Development Group
 * main.cpp  |  Flight Software  (Teensy 4.1 built-in RTC)
 *
 * MCU      : Teensy 4.1
 * Co-proc  : Holybro Kakute F405 Wing  →  Serial1 @ 115200, MAVLink
 * RTC      : Teensy 4.1 built-in (TimeLib / Teensy3Clock; VBAT coin cell)
 * Radio    : XBee 3 Pro          →  Serial6 @ 9600
 * SD       : Teensy 4.1 SDIO     →  BUILTIN_SDCARD
 *
 * MAVLink consumed: #24 GPS_RAW_INT, #27 RAW_IMU, #29 SCALED_PRESSURE,
 *                   #33 GLOBAL_POSITION_INT
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <TimeLib.h>

#include "config.h"
#include "mission_context.h"
#include "sensing.h"
#include "flight_logic.h"
#include "eeprom_store.h"
#include "servo_control.h"
#include "telemetry.h"
#include "sd_card.h"
#include "camera_ctrl.h"
#include "mavlink_handler.h"

MissionContext ctx;

uint32_t last_telem_ms  = 0;
uint32_t last_sensor_ms = 0;
uint32_t last_eeprom_ms = 0;

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void set_state(MissionState new_state) {
  if (ctx.state == new_state) return;
  ctx.state = new_state;
  eeprom_save(ctx);
  Serial.print(F("[STATE] -> "));
  Serial.println(state_name(new_state));
}


void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("[FSW] Booting..."));

  Wire.begin();

  setSyncProvider(getTeensy3Time);
  if (timeStatus() == timeSet) {
    Serial.println(F("[RTC] Teensy RTC OK"));
  } else {
    Serial.println(F("[RTC] Teensy RTC not set — awaiting ST command"));
  }

  eeprom_restore(ctx);

  MAVLINK_SERIAL.begin(MAVLINK_BAUD);
  XBEE_SERIAL.begin(XBEE_BAUD);

  // INA260 init. Non-blocking and fault-tolerant: if the chip is missing or
  // malfunctioning, power telemetry reads 0 and the loop keeps running.
  sensor_setup();

  payload_release_servo_setup(PIN_SERVO_PAYLOAD);
  egg_release_servo_setup(PIN_SERVO_EGG);
  camera_setup();

  sd_setup();
  // Header must match the packet format exactly, including the blank field
  // (,,) between CMD_ECHO and optional data (guide §3.1.1.1 #19).
  sd_write_header(
    "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,"
    "ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,CURRENT,"
    "GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,"
    "GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,"
    "CMD_ECHO,,SUBSTATE,MAIN_SOC,BUS_POWER,ACTIVE_MECHS,ACTIVE_CAMERA,MATEK"
  );

  Serial.print(F("[FSW] Ready. State: "));
  Serial.println(state_name(ctx.state));
}

void loop() {
  uint32_t now = millis();

  // 1. GCS commands — XBee uplink + USB Serial for debug
  parse_commands(ctx);
  parse_commands(ctx, Serial);

  // 2a. MAVLink — drained every loop, in every state.
  poll_mavlink(ctx);
  update_altitude_from_sim(ctx);   // no-op unless SIM mode active

  // 2b. Local sensors (INA260 + LM335) — 10 Hz, non-blocking.
  if (now - last_sensor_ms >= SENSOR_POLL_MS) {
    last_sensor_ms = now;
    read_local_sensors(ctx.sd);

    // Landing evaluated ONCE per sample — LANDING_CONFIRM_COUNT = real samples.
    ctx.sd.is_grounded = grounded_detected(ctx);

    // Apogee tracking on the sensor tick with descent margin (not per loop).
    if (ctx.state == MissionState::ASCENT) {
      if (ctx.sd.altitude_m > ctx.apogee_m) {
        ctx.apogee_m   = ctx.sd.altitude_m;
        ctx.desc_count = 0;
      } else if (ctx.sd.altitude_m < ctx.apogee_m - APOGEE_DESCENT_MARGIN_M) {
        ctx.desc_count++;
      }
    }
  }

  // 3. Telemetry — 1 Hz
  if (now - last_telem_ms >= TELEM_INTERVAL_MS) {
    send_telemetry_packet(ctx);
    last_telem_ms = now;
  }

  // 4. Periodic EEPROM flush
  if (now - last_eeprom_ms >= EEPROM_SAVE_MS) {
    eeprom_save(ctx);
    last_eeprom_ms = now;
  }

  // State machine — uses cached ctx.sd.is_grounded (no per-loop counting)
  switch (ctx.state) {

    case MissionState::LAUNCH_PAD_DISARMED:
      break;

    case MissionState::LAUNCH_PAD_ARMED:
      if (launch_detected(ctx)) {
        camera_release_start();
        camera_ground_start();
        set_state(MissionState::ASCENT);
      }
      break;

    case MissionState::ASCENT:
      if (ctx.desc_count >= APOGEE_CONFIRM_COUNT) {
        set_state(MissionState::APOGEE);
      }
      break;

    case MissionState::APOGEE:
      set_state(MissionState::DESCENT_PRE_PAYLOAD_RELEASE);
      break;

    case MissionState::DESCENT_PRE_PAYLOAD_RELEASE:
      if (ctx.sd.is_grounded) { set_state(MissionState::GROUNDED); break; }
      if ((ctx.apogee_m > 0.0f) &&
          (ctx.sd.altitude_m <= PAYLOAD_RELEASE_FRAC * ctx.apogee_m) &&
          !(ctx.flags & FLAG_PAYLOAD_RELEASED)) {
        set_state(MissionState::DESCENT_PAYLOAD_RELEASE);
      }
      break;

    case MissionState::DESCENT_PAYLOAD_RELEASE:
      if (!(ctx.flags & FLAG_PAYLOAD_RELEASED)) {
        payload_release_actuate();
        ctx.flags |= FLAG_PAYLOAD_RELEASED;
        eeprom_save(ctx);
        Serial.println(F("[MECH] Payload released"));
      }
      set_state(MissionState::DESCENT_PRE_PROBE_RELEASE);
      break;

    case MissionState::DESCENT_PRE_PROBE_RELEASE:
      if (ctx.sd.is_grounded) { set_state(MissionState::GROUNDED); break; }
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
      if (ctx.sd.is_grounded) {
        set_state(MissionState::GROUNDED);
      }
      break;

    case MissionState::GROUNDED:
      break;

    case MissionState::FAULT:
      break;

    default:
      Serial.println(F("[FSW] CRITICAL: unknown state — entering FAULT"));
      set_state(MissionState::FAULT);
      break;
  }
}
