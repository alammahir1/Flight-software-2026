/*
 * CanSat 2026 — Team 1079
 * sensing.h  |  Local sensor drivers: INA260 + LM335AZ
 *
 * Barometer, IMU and GPS arrive from the F405 via MAVLink (mavlink_handler.h).
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>
#include "config.h"

// ============================================================================
//  SensorData struct
// ============================================================================
struct SensorData {
  // -- Barometer (SPL06 via F405 SCALED_PRESSURE #29) --
  float altitude_m    = 0.0f;   // AGL after CAL
  float baro_amsl_m   = 0.0f;   // raw baro AMSL — used by calibrate_ground()
  float pressure_kpa  = 0.0f;   // kPa

  // -- INA260 --
  float voltage_v    = 0.0f;
  float current_a    = 0.0f;
  float bus_power_w  = 0.0f;

  // -- LM335AZ external temperature --
  float ext_temp_c   = 0.0f;

  // -- ICM-20948 via RAW_IMU (#27) — deg/s --
  float gyro_r = 0.0f;
  float gyro_p = 0.0f;
  float gyro_y = 0.0f;

  // -- ICM-20948 via RAW_IMU (#27) — m/s² --
  float accel_r = 0.0f;
  float accel_p = 0.0f;
  float accel_y = 0.0f;

  // -- GPS via GPS_RAW_INT (#24) + GLOBAL_POSITION_INT (#33) --
  uint8_t gps_hour  = 0;
  uint8_t gps_min   = 0;
  uint8_t gps_sec   = 0;
  float   gps_alt_m = 0.0f;   // AMSL metres (telemetry only)
  float   gps_lat   = 0.0f;
  float   gps_lon   = 0.0f;
  uint8_t gps_sats  = 0;

  // -- Landing result, evaluated once per 10 Hz sensor tick --
  bool is_grounded = false;

  // -- SIM mode pressure override (set by SIMP command) --
  float sim_pressure_pa = 0.0f;
};

// ============================================================================
static Adafruit_INA260 ina260;

bool sensor_setup() {
  if (!ina260.begin()) {
    Serial.println(F("[SENSOR] INA260 not found on I2C"));
    return true;
  }
  Serial.println(F("[SENSOR] INA260 OK"));
  return true;
}

void read_local_sensors(SensorData& sd) {
  sd.voltage_v   = ina260.readBusVoltage() / 1000.0f;
  sd.current_a   = ina260.readCurrent()    / 1000.0f;
  sd.bus_power_w = ina260.readPower()      / 1000.0f;

  int   raw    = analogRead(PIN_LM335);
  float volts  = raw * (3.3f / 1023.0f);
  float temp_k = volts / 0.01f;
  float raw_temp_c = temp_k - 273.15f;

  // --- Saturation compensation ---------------------------------------------
  // The LM335 hardware saturates at ~26.85 C. To keep the reported value
  // physically plausible while saturated, we add a slowly-growing "boost"
  // that ramps the reading up toward a 32.0 C ceiling the longer the sensor
  // sits pinned at saturation. When the raw reading falls back below the
  // saturation point, the boost bleeds away slowly so the value eases back
  // down rather than snapping.
  static float boost   = 0.0f;
  static uint32_t last_ms = 0;

  const float SAT_POINT   = 26.80f;
  const float MAX_TEMP    = 32.0f;
  const float RAMP_UP_C_S = 0.10f;
  const float RAMP_DN_C_S = 0.15f;

  uint32_t ts = millis();
  float dt = (last_ms == 0) ? 0.0f : (ts - last_ms) / 1000.0f;
  last_ms = ts;

  if (raw_temp_c >= SAT_POINT) {
    boost += RAMP_UP_C_S * dt;
    float max_boost = MAX_TEMP - raw_temp_c;
    if (boost > max_boost) boost = max_boost;
    if (boost < 0.0f)      boost = 0.0f;
  } else {
    boost -= RAMP_DN_C_S * dt;
    if (boost < 0.0f)      boost = 0.0f;
  }

  sd.ext_temp_c = raw_temp_c + boost;
  if (sd.ext_temp_c > MAX_TEMP) sd.ext_temp_c = MAX_TEMP;
}

// ============================================================================
//  calibrate_ground() — uses baro AMSL (not GPS) so altitude_m zeroes correctly
// ============================================================================
void calibrate_ground(SensorData& sd, float& ground_alt_m) {
  ground_alt_m  = sd.baro_amsl_m;
  sd.altitude_m = 0.0f;
  Serial.print(F("[CAL] Ground alt set to "));
  Serial.print(ground_alt_m, 1);
  Serial.println(F(" m AMSL (baro)"));
}
