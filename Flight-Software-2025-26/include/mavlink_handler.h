/*
 * CanSat 2026 — Team 1079
 * mavlink_handler.h  |  MAVLink polling and SIM altitude override
 *
 * MAVLink messages consumed from Matek F405 on Serial1:
 *   #24  GPS_RAW_INT          satellites_visible, time_usec (UTC)
 *   #27  RAW_IMU              xacc/yacc/zacc (mg→m/s²), xgyro/ygyro/zgyro (mrad/s→deg/s)
 *   #29  SCALED_PRESSURE      press_abs (hPa) → baro altitude AGL
 *   #33  GLOBAL_POSITION_INT  lat/lon/alt — telemetry only
 */

#pragma once
#include "mission_context.h"
#include "config.h"
#include <ardupilotmega/mavlink.h>

static mavlink_message_t mav_msg;
static mavlink_status_t  mav_status;

static uint32_t last_mav_rx_ms    = 0;
static uint32_t prev_mav_rx_count = 0;

// Conversion constants
static const float MG_TO_MS2      = 9.80665f / 1000.0f;   // mg → m/s²
static const float MRADS_TO_DEGS  = (1.0f / 1000.0f) * (180.0f / PI);  // mrad/s → deg/s

// ============================================================================
void poll_mavlink(MissionContext& ctx) {
  while (MAVLINK_SERIAL.available()) {
    uint8_t b = (uint8_t)MAVLINK_SERIAL.read();
    if (!mavlink_parse_char(MAVLINK_COMM_0, b, &mav_msg, &mav_status)) continue;

    if (mav_status.packet_rx_success_count != prev_mav_rx_count) {
      prev_mav_rx_count = mav_status.packet_rx_success_count;
      last_mav_rx_ms    = millis();
    }

    switch (mav_msg.msgid) {

      // MSG #24 — GPS_RAW_INT -----------------------------------------------
      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&mav_msg, &gps);
        ctx.sd.gps_sats = gps.satellites_visible;
        // time_usec is GPS epoch microseconds; extract UTC HH:MM:SS
        if (gps.time_usec > 0) {
          uint32_t utc_sec  = (uint32_t)((gps.time_usec / 1000000ULL) % 86400ULL);
          ctx.sd.gps_hour   = (uint8_t)(utc_sec / 3600);
          ctx.sd.gps_min    = (uint8_t)((utc_sec % 3600) / 60);
          ctx.sd.gps_sec    = (uint8_t)(utc_sec % 60);
        }
        break;
      }

      // MSG #27 — RAW_IMU ---------------------------------------------------
      case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t raw;
        mavlink_msg_raw_imu_decode(&mav_msg, &raw);
        // Accel: mg → m/s²
        ctx.sd.accel_r = (float)raw.xacc * MG_TO_MS2;
        ctx.sd.accel_p = (float)raw.yacc * MG_TO_MS2;
        ctx.sd.accel_y = (float)raw.zacc * MG_TO_MS2;
        // Gyro: mrad/s → deg/s
        ctx.sd.gyro_r  = (float)raw.xgyro * MRADS_TO_DEGS;
        ctx.sd.gyro_p  = (float)raw.ygyro * MRADS_TO_DEGS;
        ctx.sd.gyro_y  = (float)raw.zgyro * MRADS_TO_DEGS;
        break;
      }

      // MSG #29 — SCALED_PRESSURE -------------------------------------------
      case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t sp;
        mavlink_msg_scaled_pressure_decode(&mav_msg, &sp);
        float pressure_pa      = sp.press_abs * 100.0f;   // hPa → Pa
        ctx.sd.pressure_kpa    = pressure_pa / 1000.0f;
        float amsl             = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
        ctx.sd.baro_amsl_m     = amsl;                    // raw baro AMSL for CAL
        ctx.sd.altitude_m      = amsl - ctx.ground_alt_m; // AGL
        break;
      }

      // MSG #33 — GLOBAL_POSITION_INT — telemetry only ----------------------
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gpi;
        mavlink_msg_global_position_int_decode(&mav_msg, &gpi);
        ctx.sd.gps_lat   = (float)gpi.lat / 10000000.0f;
        ctx.sd.gps_lon   = (float)gpi.lon / 10000000.0f;
        ctx.sd.gps_alt_m = (float)gpi.alt / 1000.0f;
        // altitude_m is NOT updated here — baro is the authoritative source
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
  float pressure_pa      = ctx.sd.sim_pressure_pa;
  ctx.sd.pressure_kpa    = pressure_pa / 1000.0f;
  float amsl             = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
  ctx.sd.baro_amsl_m     = amsl;                    // keep in sync so CAL works in SIM mode
  ctx.sd.altitude_m      = amsl - ctx.ground_alt_m;
}

// ============================================================================
uint8_t matek_heartbeat_ok() {
  return ((millis() - last_mav_rx_ms) < 2000u) ? 1 : 0;
}
