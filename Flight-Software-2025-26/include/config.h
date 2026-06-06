/*
 * CanSat 2026 — Team 1079
 * config.h  |  Compile-time constants, pin assignments, thresholds
 */

#pragma once
#include <Arduino.h>

// ============================================================================
//  Team / Mission identity
// ============================================================================
#define TEAM_ID_STR   "1079"
#define TEAM_ID_INT   1079

// ============================================================================
//  Serial ports
// ============================================================================
#define MAVLINK_SERIAL   Serial1   // Matek F405   RX=0  TX=1
#define XBEE_SERIAL      Serial6   // XBee 3 Pro   RX=6  TX=7
#define MAVLINK_BAUD     115200
#define XBEE_BAUD        9600

// ============================================================================
//  Pin assignments  (Teensy 4.1)
// ============================================================================
const int PIN_SERVO_PAYLOAD = 9;    // payload / container separation
const int PIN_SERVO_EGG     = 10;   // egg release (Emax ES3053)
const int PIN_LM335         = 14;   // LM335AZ external temperature

// ============================================================================
//  Mission thresholds  — TUNE after bench/drop testing
// ============================================================================

// Launch detection
const float LAUNCH_DETECT_ALT_M    = 10.0f;   // metres AGL to confirm launch
const float LAUNCH_ACCEL_THRESHOLD = 20.0f;   // m/s² net upward (~2g). accel_y
                                              // is now SI (m/s²), NOT raw counts.

// Apogee detection
const int   APOGEE_CONFIRM_COUNT    = 3;      // consecutive descending samples
const float APOGEE_DESCENT_MARGIN_M = 1.0f;   // buffer vs baro/GPS jitter

// Payload release
const float PAYLOAD_RELEASE_FRAC   = 0.80f;   // C5: 80% of apogee altitude

// Egg / probe release
// NOTE: C12 requires 2.0 m ± 0.5 m. 7.0 m is a deliberate team override as an
// actuation-latency safety net — this is OUTSIDE the C12 tolerance and will not
// score the egg-release requirement. Revert to 2.0f for competition compliance.
const float EGG_RELEASE_ALT_M      = 7.0f;    // metres AGL  (C12 spec = 2.0 m)

// Landing detection  (accel_y in m/s², 1g = 9.81)
const float LANDING_ALT_M          = 0.5f;    // metres AGL
const float GROUNDED_ACCEL_MIN     = 8.0f;    // m/s²  (1g − tolerance)
const float GROUNDED_ACCEL_MAX     = 11.5f;   // m/s²  (1g + tolerance)
const int   LANDING_CONFIRM_COUNT  = 2;       // consecutive SAMPLES (10 Hz tick)

// ============================================================================
//  Timing
// ============================================================================
const uint32_t TELEM_INTERVAL_MS   = 1000;    // 1 Hz telemetry
const uint32_t EEPROM_SAVE_MS      = 5000;    // periodic EEPROM flush
const uint32_t SENSOR_POLL_MS      = 100;     // 10 Hz INA260 non-blocking

// Camera trigger pins (digital HIGH = record, LOW = stop)
const int PIN_CAM_RELEASE = 5;
const int PIN_CAM_GROUND  = 4;
