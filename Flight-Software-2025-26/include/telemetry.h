/*
 * CanSat 2026 — Team 1079
 * telemetry.h  |  Packet transmission and uplink command parsing
 *
 * Telemetry CSV (1 Hz, '\r' terminated) — competition-spec field order:
 *   TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE,
 *   PRESSURE, VOLTAGE, CURRENT, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P,
 *   ACCEL_Y, GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS,
 *   CMD_ECHO  [, OPTIONAL_DATA...]
 *
 * Time comes from the Teensy 4.1 built-in RTC via TimeLib (synced in main.cpp).
 */

#pragma once
#include <TimeLib.h>          // Teensy built-in RTC (replaces RV-3028)

#include "mission_context.h"
#include "config.h"
#include "eeprom_store.h"
#include "sd_card.h"
#include "sensing.h"
#include "servo_control.h"
#include "camera_ctrl.h"
#include "mavlink_handler.h"

// ============================================================================
//  Set the Teensy built-in RTC (and TimeLib software clock) to hh:mm:ss.
//  Date is fixed to a placeholder — only time-of-day is telemetered.
// ============================================================================
static void set_teensy_time(uint8_t hh, uint8_t mm, uint8_t ss) {
  setTime(hh, mm, ss, 1, 1, 2025);   // h, m, s, day, month, year
  Teensy3Clock.set(now());           // persist to hardware RTC (needs VBAT cell)
}

// ============================================================================
//  send_telemetry_packet
// ============================================================================
void send_telemetry_packet(MissionContext& ctx) {
  // MISSION_TIME — UTC hh:mm:ss from the Teensy RTC
  char mission_time[12];
  snprintf(mission_time, sizeof(mission_time),
           "%02d:%02d:%02d", hour(), minute(), second());

  // GPS_TIME — from MAVLink GPS_RAW_INT (#24)
  char gps_time[12];
  snprintf(gps_time, sizeof(gps_time),
           "%02u:%02u:%02u",
           ctx.sd.gps_hour, ctx.sd.gps_min, ctx.sd.gps_sec);

  // Optional ACTIVE_MECHS bitmask
  uint8_t active_mechs = 0;
  if (ctx.flags & FLAG_PAYLOAD_RELEASED) active_mechs |= 0x01;
  if (ctx.flags & FLAG_PROBE_RELEASED)   active_mechs |= 0x02;

  char buf[384];
  snprintf(buf, sizeof(buf),
    // ---- Required spec fields ----
    "%s,"               // 1  TEAM_ID
    "%s,"               // 2  MISSION_TIME
    "%lu,"              // 3  PACKET_COUNT
    "%c,"               // 4  MODE  F/S
    "%s,"               // 5  STATE (official)
    "%.1f,"             // 6  ALTITUDE      m AGL   (0.1 m)
    "%.1f,"             // 7  TEMPERATURE   °C      (0.1)
    "%.1f,"             // 8  PRESSURE      kPa     (0.1)
    "%.1f,"             // 9  VOLTAGE       V       (0.1)
    "%.2f,"             // 10 CURRENT       A       (0.01)
    "%.2f,%.2f,%.2f,"   // 11 GYRO_R,P,Y    °/s
    "%.2f,%.2f,%.2f,"   // 12 ACCEL_R,P,Y
    "%s,"               // 13 GPS_TIME
    "%.1f,"             // 14 GPS_ALTITUDE  m AMSL  (0.1 m)
    "%.4f,"             // 15 GPS_LATITUDE  deg     (0.0001)
    "%.4f,"             // 16 GPS_LONGITUDE deg     (0.0001)
    "%u,"               // 17 GPS_SATS      int
    "%s"                // 18 CMD_ECHO      (no commas)
    // ---- Optional data (after CMD_ECHO, allowed by spec) ----
    ",%s,%.1f,%.2f,%u,%u,%u\r",  // SUBSTATE, MAIN_SOC, BUS_POWER,
                                 // ACTIVE_MECHS, ACTIVE_CAMERA, MATEK
    TEAM_ID_STR,
    mission_time,
    (unsigned long)ctx.packet_count,
    ctx.mode_char(),
    telem_state_name(ctx.state),
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
    state_name(ctx.state),      // SUBSTATE = detailed internal state
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

void parse_commands(MissionContext& ctx) {
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
          set_teensy_time(ctx.sd.gps_hour, ctx.sd.gps_min, ctx.sd.gps_sec);
        } else {
          int hh = 0, mm = 0, ss = 0;
          if (sscanf(p, "%d:%d:%d", &hh, &mm, &ss) == 3) {
            set_teensy_time((uint8_t)hh, (uint8_t)mm, (uint8_t)ss);
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
