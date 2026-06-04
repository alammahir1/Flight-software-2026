/*
 * CanSat 2026 — Team 1079
 * mavlink_handler.h  |  MAVLink polling and SIM altitude override
 *
 * Consumes MAVLink messages from the Matek F405 on Serial1:
 *   #24  GPS_RAW_INT          satellites_visible, time_usec (UTC)
 *   #27  RAW_IMU              xacc, yacc, zacc (raw counts)
 *   #29  SCALED_PRESSURE      press_abs (hPa), temperature (cdegC) — primary altitude source
 *   #30  ATTITUDE             roll/pitch/yaw (rad → deg)
 *   #33  GLOBAL_POSITION_INT  lat/lon (/1e7 → °), GPS AMSL alt (/1000 → m)
 *
 * altitude_m is driven by SCALED_PRESSURE (barometric) via the hypsometric
 * formula. GPS altitude is stored separately for telemetry but is NOT used
 * for flight decisions — GPS altitude is too noisy and slow for apogee/landing.
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

    // Update heartbeat tracker on any successful packet
    if (mav_status.packet_rx_success_count != prev_mav_rx_count) {
      prev_mav_rx_count = mav_status.packet_rx_success_count;
      last_mav_rx_ms    = millis();
    }

    switch (mav_msg.msgid) {

      // MSG #24 — GPS_RAW_INT ------------------------------------------------
      // Source for satellite count and GPS UTC time.
      // time_usec is microseconds since UNIX epoch (GPS week rollover-corrected
      // by ArduPilot). We extract HH:MM:SS of the current UTC day.
      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gri;
        mavlink_msg_gps_raw_int_decode(&mav_msg, &gri);
        ctx.sd.gps_sats = gri.satellites_visible;
        if (gri.time_usec > 0) {
          uint32_t secs_of_day = (uint32_t)((gri.time_usec / 1000000ULL) % 86400ULL);
          ctx.sd.gps_hour = (uint8_t)(secs_of_day / 3600);
          ctx.sd.gps_min  = (uint8_t)((secs_of_day % 3600) / 60);
          ctx.sd.gps_sec  = (uint8_t)(secs_of_day % 60);
        }
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

      // MSG #29 — SCALED_PRESSURE -------------------------------------------
      // Primary altitude source. press_abs is in hPa; convert to Pa for the
      // hypsometric formula. Temperature is in centidegrees C.
      // In SIM mode this is overridden by update_altitude_from_sim().
      case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t sp;
        mavlink_msg_scaled_pressure_decode(&mav_msg, &sp);
        float pressure_pa   = sp.press_abs * 100.0f;          // hPa → Pa
        ctx.sd.pressure_kpa = pressure_pa / 1000.0f;          // Pa → kPa for telemetry
        // Hypsometric formula: altitude AMSL in metres
        float amsl          = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
        ctx.sd.altitude_m   = amsl - ctx.ground_alt_m;        // AMSL → AGL
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

      // MSG #33 — GLOBAL_POSITION_INT ---------------------------------------
      // GPS lat/lon/alt stored for telemetry only.
      // altitude_m (used for flight decisions) comes from SCALED_PRESSURE above.
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gpi;
        mavlink_msg_global_position_int_decode(&mav_msg, &gpi);
        ctx.sd.gps_lat   = (float)gpi.lat / 10000000.0f;
        ctx.sd.gps_lon   = (float)gpi.lon / 10000000.0f;
        ctx.sd.gps_alt_m = (float)gpi.alt / 1000.0f;
        break;
      }

      default: break;
    }
  }
}

// ============================================================================
//  SIM mode altitude override
//  Hypsometric formula: h_amsl = 44330 × (1 − (P/P₀)^0.1903)
//  Overwrites the barometric altitude_m and pressure_kpa set by SCALED_PRESSURE.
// ============================================================================
void update_altitude_from_sim(MissionContext& ctx) {
  if (!ctx.sim_active()) return;
  float pressure_pa       = ctx.sd.sim_pressure_pa;
  float amsl              = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
  ctx.sd.altitude_m       = amsl - ctx.ground_alt_m;
  ctx.sd.pressure_kpa     = pressure_pa / 1000.0f;
}

// ============================================================================
uint8_t matek_heartbeat_ok() {
  return ((millis() - last_mav_rx_ms) < 2000u) ? 1 : 0;
}
