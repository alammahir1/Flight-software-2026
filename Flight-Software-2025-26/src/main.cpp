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
 *   #24  GPS_RAW_INT          satellites_visible, UTC time
 *   #27  RAW_IMU              xacc, yacc, zacc (raw counts)
 *   #29  SCALED_PRESSURE      press_abs (hPa) — primary altitude source
 *   #30  ATTITUDE             roll/pitch/yaw (rad → deg)
 *   #33  GLOBAL_POSITION_INT  lat/lon/alt (telemetry only)
 *
 * Telemetry packet (1 Hz CSV, '\r' terminated):
 *   TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE,
 *   ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT,
 *   GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
 *   GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
 *   CMD_ECHO, SUBSTATE, MAIN_SOC, BUS_POWER, ACTIVE_MECHS,
 *   ACTIVE_CAMERA, MATEK
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RV-3028-C7.h>

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

//  Globals — one context object and the RTC. That's it.
MissionContext ctx;
RV3028         rtc;

//  Timing
uint32_t last_telem_ms  = 0;
uint32_t last_sensor_ms = 0;
uint32_t last_eeprom_ms = 0;

//  Helper: change state, log it, save to EEPROM
void set_state(MissionState new_state) {
  if (ctx.state == new_state) return;
  ctx.state = new_state;
  eeprom_save(ctx);
  Serial.print(F("[STATE] -> "));
  Serial.println(state_name(new_state));
}

// NOTE: is_pre_launch() has been removed. It was only ever used to gate
// sensor polling, which was the lockout bug. Now that poll_mavlink() runs
// unconditionally every loop, there is nothing left that needs to test for
// pre-launch as a combined condition — the state machine handles each state
// individually. Keeping a dead helper that was the root cause of a critical
// bug would be confusing.


void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("[FSW] Booting..."));

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println(F("[RTC] RV3028 not found — awaiting ST command"));
  } else {
    rtc.set24Hour();
    Serial.println(F("[RTC] RV3028 OK"));
  }

  eeprom_restore(ctx);

  MAVLINK_SERIAL.begin(MAVLINK_BAUD);
  XBEE_SERIAL.begin(XBEE_BAUD);

  while (!sensor_setup()) {
    Serial.println(F("[SENSOR] Init failed, retrying..."));
    delay(500);
  }

  payload_release_servo_setup(PIN_SERVO_PAYLOAD);
  egg_release_servo_setup(PIN_SERVO_EGG);
  camera_setup();

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

  // 1. GCS commands — always
  parse_commands(ctx, rtc);

  // 2a. MAVLink — drained every loop, in every state.
  //     This is the only source of accel_y / altitude_m. It is a non-blocking
  //     UART drain and must never be gated by state — doing so was the lockout
  //     bug that prevented launch detection on the pad.
  poll_mavlink(ctx);
  update_altitude_from_sim(ctx);   // no-op unless SIM mode active

  // 2b. Local sensors: INA260 (I2C) + LM335 (analog) — 10 Hz timer.
  //     These are blocking calls (~500 µs each on I2C). Scheduling them at
  //     SENSOR_POLL_MS keeps the loop fast for launch detection while still
  //     updating power/temperature data fast enough for 1 Hz telemetry.
  if (now - last_sensor_ms >= SENSOR_POLL_MS) {
    last_sensor_ms = now;
    read_local_sensors(ctx.sd);

    // Apogee tracking is evaluated here — on the sensor tick — NOT every loop
    // iteration. Evaluating it every loop caused desc_count to blow past
    // APOGEE_CONFIRM_COUNT in milliseconds on a single GPS wobble, triggering
    // false apogee detection. Tying it to the sensor tick means desc_count
    // increments at most 10 times per second, matching actual sensor cadence.
    if (ctx.state == MissionState::ASCENT) {
      if (ctx.sd.altitude_m > ctx.apogee_m) {
        ctx.apogee_m   = ctx.sd.altitude_m;
        ctx.desc_count = 0;
      } else if (ctx.sd.altitude_m < ctx.apogee_m - APOGEE_DESCENT_MARGIN_M) {
        // Only count descent once we've dropped at least APOGEE_DESCENT_MARGIN_M
        // below the peak — filters GPS/baro jitter at apogee.
        ctx.desc_count++;
      }
    }
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
        camera_release_start();
        camera_ground_start();
        set_state(MissionState::ASCENT);
      }
      break;

    case MissionState::ASCENT:
      // Apogee tracking runs in the sensor tick above.
      if (grounded_detected(ctx)) {
        set_state(MissionState::GROUNDED);
        break;
      }
      if (ctx.desc_count >= APOGEE_CONFIRM_COUNT) {
        set_state(MissionState::APOGEE);
      }
      break;

    case MissionState::APOGEE:
      set_state(MissionState::DESCENT_PRE_PAYLOAD_RELEASE);
      break;

    case MissionState::DESCENT_PRE_PAYLOAD_RELEASE:
      if (grounded_detected(ctx)) { set_state(MissionState::GROUNDED); break; }
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
      break;

    case MissionState::FAULT:
      break;

    default:
      Serial.println(F("[FSW] CRITICAL ERROR: unknown state — reverting to ASCENT"));
      ctx.desc_count = 0;
      set_state(MissionState::ASCENT);
      break;
  }
}
