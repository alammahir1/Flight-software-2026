/*
 * CanSat 2026 — Team 1079
 * config.h  |  Compile-time constants, pin assignments, thresholds
 *
 * Edit this file to tune thresholds after bench and drop testing.
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
#define XBEE_SERIAL      Serial6   // XBee 3 Pro   RX=6  TX=6
#define MAVLINK_BAUD     115200
#define XBEE_BAUD        9600

// ============================================================================
//  Pin assignments  (Teensy 4.1)
// ============================================================================
const int PIN_SERVO_PAYLOAD = 9;    // payload / container separation
const int PIN_SERVO_EGG     = 10;   // egg release  (Emax ES3053)
const int PIN_LM335         = 14;   // LM335AZ external temperature

// ============================================================================
//  Mission thresholds  — TUNE after bench/drop testing
// ============================================================================

// Launch detection
const float LAUNCH_DETECT_ALT_M    = 10.0f;   // metres AGL to confirm launch
const float LAUNCH_ACCEL_THRESHOLD = 3000.0f; // ICM-20948 raw upward counts
                                               // (~1.5g — verify scale on bench)

// Apogee detection
const int   APOGEE_CONFIRM_COUNT     = 3;      // consecutive descending sensor samples
const float APOGEE_DESCENT_MARGIN_M  = 1.0f;  // must drop this far below peak before
                                               // desc_count increments — filters GPS jitter

// Payload release
const float PAYLOAD_RELEASE_FRAC   = 0.80f;   // 80 % of apogee altitude

// Egg / probe release
const float EGG_RELEASE_ALT_M      = 7.0f;   // metres AGL

// Landing detection
const float LANDING_ALT_M          = 0.5f;    // metres AGL
const float GROUNDED_ACCEL_MIN     = 1800.0f; // raw counts: 1g − tolerance
const float GROUNDED_ACCEL_MAX     = 2300.0f; // raw counts: 1g + tolerance
const int   LANDING_CONFIRM_COUNT  = 2;        // consecutive readings

// ============================================================================
//  Timing
// ============================================================================
const uint32_t TELEM_INTERVAL_MS   = 1000;   // 1 Hz telemetry
const uint32_t EEPROM_SAVE_MS      = 5000;   // periodic EEPROM flush
const uint32_t SENSOR_POLL_MS      = 100;    // 10 Hz local sensor poll (INA260 + LM335)
                                             // MAVLink is drained every loop regardless

// Camera trigger pins (digital HIGH = record, LOW = stop)
const int PIN_CAM_RELEASE = 5;
const int PIN_CAM_GROUND  = 4;
