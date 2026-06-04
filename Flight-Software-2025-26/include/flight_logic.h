/*
 * CanSat 2026 — Team 1079
 * flight_logic.h  |  Launch and grounded detection
 *
 * IMPORTANT: grounded_detected() must be called ONCE PER SENSOR TICK (10 Hz),
 * not every loop iteration — its internal counter debounces by SAMPLE count.
 * Calling it every loop makes LANDING_CONFIRM_COUNT meaningless.
 *
 * accel_y is in m/s² (SI) — thresholds in config.h are m/s², not raw counts.
 */

#pragma once
#include "config.h"
#include "mission_context.h"

// ============================================================================
//  Launch detection — TRUE if altitude OR upward accel exceeds threshold.
// ============================================================================
bool launch_detected(const MissionContext& ctx) {
  bool alt_trigger   = ctx.sd.altitude_m > LAUNCH_DETECT_ALT_M;
  bool accel_trigger = ctx.sd.accel_y    > LAUNCH_ACCEL_THRESHOLD;
  return alt_trigger || accel_trigger;
}

// ============================================================================
//  Grounded detection — TRUE after LANDING_CONFIRM_COUNT consecutive SAMPLES
//  with altitude below LANDING_ALT_M AND accel in the static 1g band.
//  Call exactly once per 10 Hz sensor tick.
// ============================================================================
bool grounded_detected(MissionContext& ctx) {
  static uint8_t gnd_count = 0;

  bool alt_low  = ctx.sd.altitude_m < LANDING_ALT_M;
  bool accel_1g = (ctx.sd.accel_y > GROUNDED_ACCEL_MIN) &&
                  (ctx.sd.accel_y < GROUNDED_ACCEL_MAX);

  if (alt_low && accel_1g) {
    if (gnd_count < 255) gnd_count++;
  } else {
    gnd_count = 0;
  }

  return gnd_count >= LANDING_CONFIRM_COUNT;
}
