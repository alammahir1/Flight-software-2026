/*
 * CanSat 2026 — Team 1079
 * telemetry.h  |  Packet transmission and uplink command parsing
 *
 * Telemetry CSV (1 Hz, '\r' terminated) — competition-spec field order:
 *   TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE,
 *   PRESSURE, VOLTAGE, CURRENT, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P,
 *   ACCEL_Y, GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
 *   CMD_ECHO  [,OPTIONAL_DATA]
 *
 * Commands handled (per competition spec):
 *   CMD,1079,CX,ON|OFF
 *   CMD,1079,ST,hh:mm:ss|GPS
 *   CMD,1079,CAL
 *   CMD,1079,SIM,ENABLE|ACTIVATE|DISABLE
 *   CMD,1079,SIMP,<pressure_Pa>
 *   CMD,1079,MEC,<DEVICE>,ON|OFF
 *     DEVICE: ARM  (DISARMED→ARMED), PAYLOAD, PROBE
 *
 * Time source: Teensy 4.1 built-in RTC via TimeLib (no external RTC).
 */

#pragma once
#include <TimeLib.h>

#include "mission_context.h"
#include "config.h"
#include "eeprom_store.h"
#include "sd_card.h"
#include "sensing.h"
#include "servo_control.h"
#include "camera_ctrl.h"
#include "mavlink_handler.h"

// ============================================================================
//  Set the Teensy 4.1 built-in RTC and sync TimeLib to it.
//  Needs a coin cell on the VBAT pad to persist across power-off.
// ============================================================================
static void set_teensy_time(uint8_t hh, uint8_t mm, uint8_t ss) {
  setTime(hh, mm, ss, 1, 1, 2025);   // h, m, s, day, month, year (date unused)
  Teensy3Clock.set(now());            // write to hardware RTC
}

// ============================================================================
//  send_telemetry_packet — competition-spec CSV
// ============================================================================
void send_telemetry_packet(MissionContext& ctx) {
  // MISSION_TIME from Teensy RTC
  char mission_time[12];
  snprintf(mission_time, sizeof(mission_time),
           "%02d:%02d:%02d", hour(), minute(), second());

  // GPS_TIME from MAVLink GPS_RAW_INT (#24)
  char gps_time[12];
  snprintf(gps_time, sizeof(gps_time),
           "%02u:%02u:%02u",
           ctx.sd.gps_hour, ctx.sd.gps_min, ctx.sd.gps_sec);

  // Optional ACTIVE_MECHS bitmask (after required CMD_ECHO field)
  uint8_t active_mechs = 0;
  if (ctx.flags & FLAG_PAYLOAD_RELEASED) active_mechs |= 0x01;
  if (ctx.flags & FLAG_PROBE_RELEASED)   active_mechs |= 0x02;

  char buf[384];
  snprintf(buf, sizeof(buf),
    // ---- Required spec fields (1–18) ----
    "%s,"               // 1  TEAM_ID
    "%s,"               // 2  MISSION_TIME      hh:mm:ss UTC
    "%lu,"              // 3  PACKET_COUNT
    "%c,"               // 4  MODE              F or S
    "%s,"               // 5  STATE             official string
    "%.1f,"             // 6  ALTITUDE          m AGL    (0.1 m)
    "%.1f,"             // 7  TEMPERATURE       °C       (0.1)
    "%.1f,"             // 8  PRESSURE          kPa      (0.1)
    "%.1f,"             // 9  VOLTAGE           V        (0.1)
    "%.2f,"             // 10 CURRENT           A        (0.01)
    "%.2f,%.2f,%.2f,"   // 11 GYRO_R,P,Y        deg/s
    "%.2f,%.2f,%.2f,"   // 12 ACCEL_R,P,Y       m/s²
    "%s,"               // 13 GPS_TIME          hh:mm:ss UTC
    "%.1f,"             // 14 GPS_ALTITUDE      m AMSL   (0.1 m)
    "%.4f,"             // 15 GPS_LATITUDE      deg N    (0.0001)
    "%.4f,"             // 16 GPS_LONGITUDE     deg W    (0.0001)
    "%u,"               // 17 GPS_SATS          integer
    "%s"                // 18 CMD_ECHO          no commas
    // ---- Optional data ----
    ",%s,%.2f,%u,%u,%u\r",  // SUBSTATE, BUS_POWER, ACTIVE_MECHS, ACTIVE_CAM, MATEK
    TEAM_ID_STR,
    mission_time,
    (unsigned long)ctx.packet_count,
    ctx.mode_char(),
    telem_state_name(ctx.state),   // official STATE string
    ctx.sd.altitude_m,
    ctx.sd.ext_temp_c,
    ctx.sd.pressure_kpa,
    ctx.sd.voltage_v,
    ctx.sd.current_a,
    ctx.sd.gyro_r,  ctx.sd.gyro_p,  ctx.sd.gyro_y,
    ctx.sd.accel_r, ctx.sd.accel_p, ctx.sd.accel_y,
    gps_time,
    ctx.sd.gps_alt_m,
    ctx.sd.gps_lat,
    ctx.sd.gps_lon,
    ctx.sd.gps_sats,
    ctx.cmd_echo,
    // optional
    state_name(ctx.state),    // SUBSTATE = detailed internal state for debugging
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
//  parse_commands — drains XBee RX one line at a time
// ============================================================================
static char rx_buf[128];
static int  rx_idx = 0;

void parse_commands(MissionContext& ctx) {
  while (XBEE_SERIAL.available()) {
    char c = (char)XBEE_SERIAL.read();

    if (c == '\r' || c == '\n') {
      if (rx_idx == 0) continue;
      rx_buf[rx_idx] = '\0';
      rx_idx = 0;

      char* tok = strtok(rx_buf, ",");
      if (!tok || strcmp(tok, "CMD") != 0)   goto done;

      tok = strtok(nullptr, ",");            // TEAM_ID
      if (!tok || atoi(tok) != TEAM_ID_INT)  goto done;

      tok = strtok(nullptr, ",");            // command keyword
      if (!tok) goto done;

      // CX — Telemetry on/off -----------------------------------------------
      // CMD,<TEAM_ID>,CX,ON|OFF
      if (strcmp(tok, "CX") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "ON") == 0) {
          ctx.flags |= FLAG_CX_ON;
          ctx.packet_count = 0;    // spec: reset packet count on CX ON
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "CXON",  31);
          Serial.println(F("[CMD] CX ON"));
        } else if (strcmp(p, "OFF") == 0) {
          ctx.flags &= ~FLAG_CX_ON;
          strncpy(ctx.cmd_echo, "CXOFF", 31);
          Serial.println(F("[CMD] CX OFF"));
        }
      }

      // ST — Set time --------------------------------------------------------
      // CMD,<TEAM_ID>,ST,hh:mm:ss  or  CMD,<TEAM_ID>,ST,GPS
      else if (strcmp(tok, "ST") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "GPS") == 0) {
          set_teensy_time(ctx.sd.gps_hour, ctx.sd.gps_min, ctx.sd.gps_sec);
          Serial.println(F("[CMD] ST GPS"));
        } else {
          int hh = 0, mm = 0, ss = 0;
          if (sscanf(p, "%d:%d:%d", &hh, &mm, &ss) == 3) {
            set_teensy_time((uint8_t)hh, (uint8_t)mm, (uint8_t)ss);
            Serial.println(F("[CMD] ST manual"));
          }
        }
        strncpy(ctx.cmd_echo, "ST", 31);
      }

      // CAL — Calibrate altitude to zero ------------------------------------
      // CMD,<TEAM_ID>,CAL
      else if (strcmp(tok, "CAL") == 0) {
        calibrate_ground(ctx.sd, ctx.ground_alt_m);
        eeprom_save(ctx);
        strncpy(ctx.cmd_echo, "CAL", 31);
        Serial.println(F("[CMD] CAL"));
      }

      // SIM — Simulation mode control ---------------------------------------
      // CMD,<TEAM_ID>,SIM,ENABLE|ACTIVATE|DISABLE
      else if (strcmp(tok, "SIM") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (strcmp(p, "ENABLE") == 0) {
          ctx.flags |=  FLAG_SIM_ENABLED;
          ctx.flags &= ~FLAG_SIM_ACTIVE;   // ENABLE does not yet ACTIVATE
          strncpy(ctx.cmd_echo, "SIMENABLE", 31);
          Serial.println(F("[CMD] SIM ENABLE"));
        } else if (strcmp(p, "ACTIVATE") == 0) {
          if (ctx.flags & FLAG_SIM_ENABLED) {
            ctx.flags |= FLAG_SIM_ACTIVE;
            strncpy(ctx.cmd_echo, "SIMACTIVATE", 31);
            Serial.println(F("[CMD] SIM ACTIVATE"));
          } else {
            Serial.println(F("[CMD] SIM ACTIVATE ignored — not ENABLED"));
          }
        } else if (strcmp(p, "DISABLE") == 0) {
          ctx.flags &= ~(FLAG_SIM_ENABLED | FLAG_SIM_ACTIVE);
          strncpy(ctx.cmd_echo, "SIMDISABLE", 31);
          Serial.println(F("[CMD] SIM DISABLE"));
        }
        eeprom_save(ctx);
      }

      // SIMP — Simulated pressure (SIM ACTIVE only) -------------------------
      // CMD,<TEAM_ID>,SIMP,<pressure_Pa>
      // CMD_ECHO format: SIMP<value>  e.g. SIMP101325
      else if (strcmp(tok, "SIMP") == 0) {
        char* p = strtok(nullptr, ",");
        if (!p) goto done;
        if (ctx.flags & FLAG_SIM_ACTIVE) {
          ctx.sd.sim_pressure_pa = (float)atol(p);
          snprintf(ctx.cmd_echo, 32, "SIMP%s", p);
          // SIM pressure is applied immediately in the next poll_mavlink() call
          // via update_altitude_from_sim()
        } else {
          Serial.println(F("[CMD] SIMP ignored — SIM not active"));
        }
      }

      // MEC — Mechanism actuation -------------------------------------------
      // CMD,<TEAM_ID>,MEC,<DEVICE>,ON|OFF
      else if (strcmp(tok, "MEC") == 0) {
        char* device = strtok(nullptr, ",");
        char* onoff  = strtok(nullptr, ",");
        if (!device || !onoff) goto done;

        if (strcmp(device, "ARM") == 0 && strcmp(onoff, "ON") == 0) {
          if (ctx.state == MissionState::LAUNCH_PAD_DISARMED) {
            ctx.state = MissionState::LAUNCH_PAD_ARMED;
            eeprom_save(ctx);
            Serial.println(F("[CMD] MEC ARM ON — ARMED"));
          } else {
            Serial.println(F("[CMD] MEC ARM ON — ignored, not DISARMED"));
          }
          strncpy(ctx.cmd_echo, "MECARM", 31);

        } else if (strcmp(device, "PAYLOAD") == 0 && strcmp(onoff, "ON") == 0) {
          payload_release_actuate();
          ctx.flags |= FLAG_PAYLOAD_RELEASED;
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "MECPAYLOAD", 31);
          Serial.println(F("[CMD] MEC PAYLOAD ON"));

        } else if (strcmp(device, "PROBE") == 0 && strcmp(onoff, "ON") == 0) {
          egg_release_actuate();
          ctx.flags |= FLAG_PROBE_RELEASED;
          eeprom_save(ctx);
          strncpy(ctx.cmd_echo, "MECPROBE", 31);
          Serial.println(F("[CMD] MEC PROBE ON"));
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
