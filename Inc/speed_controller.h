/*
 * speed_controller.h — Speed controller abstraction for AM32
 *
 * Encapsulates the speed PID state machine, target tracking, and
 * output accumulation.  The interface is designed so the underlying
 * control algorithm can be swapped without changing call sites.
 *
 * The controller operates purely in the commutation-interval domain (µs).
 * RPM ↔ µs conversion is the caller's responsibility.
 *
 * Architecture (feedforward + P + I + lowpass D):
 *   1. Feedforward maps target_e_com_time → baseline throttle (open-loop).
 *   2. P + I micro-trim around the baseline (closed-loop).
 *   3. D operates on a first-order lowpass of the error to reject BEMF noise.
 *
 * Responsibilities:
 *  - PID state machine (error, integral, derivative)
 *  - Feedforward baseline calculation
 *  - Error lowpass for derivative
 *  - Output accumulation & clamping
 *  - Integral reset during startup
 *
 * Stays outside (caller's domain):
 *  - Input arbitration (RawCommand vs RPMCommand)
 *  - Mode flags (drive_by_rpm, rpm_command_active)
 *  - Direction (forward/reverse)
 *  - RPM ↔ commutation-interval conversion
 */

#pragma once

#include <stdint.h>
#include "common.h"   // for fastPID

#ifdef __cplusplus
extern "C" {
#endif

/*
  Derivative noise floor: error must change by at least this many µs
  (commutation interval domain) before the D term activates.
  Default 10 µs — suppresses dither with zero measurable latency cost.
 */
#define SC_D_NOISE_FLOOR 10

typedef struct {
    /* PID state machine */
    fastPID pid;

    /* Target commutation interval in µs */
    uint16_t target_e_com_time;

    /*
      Feedforward baseline, scaled ×10000.
      Computed from target_e_com_time when SC_SetTarget is called.
      Maps to roughly the throttle level a motor of the given kV would
      need to reach the target RPM.
     */
    int32_t ff_baseline;

    /*
      PID output accumulator, scaled ×10000.
      Range: 0 ~ 2047×10000  (maps to throttle 48–2047 after /10000).
      In steady state this converges to ff_baseline + pi_trim.
     */
    int32_t input_override;

    /* Lowpass state for derivative */
    int32_t filtered_error;
    int32_t last_filtered_error;
} SpeedController;

/*
  Initialise the speed controller.

  Reads PID gains from the global speed_kp / speed_ki / speed_kd
  variables so they can be updated at runtime (DroneCAN param Set).
 */
void SC_Init(SpeedController *sc);

/*
  Set the target commutation interval (µs) and re-compute the
  feedforward baseline.  Call this whenever the RPM target changes.

  The caller converts RPM to µs using:
    target_us = 60000000 / RPM / (motor_poles / 2)
 */
void SC_SetTarget(SpeedController *sc, uint16_t target_e_com_time);

/*
  Set the feedforward baseline directly (×10000 scale).
  Alternative to SC_SetTarget for callers that want to bypass the
  default feedforward calculation.
 */
void SC_SetFeedforward(SpeedController *sc, int32_t ff_baseline);

/*
  1kHz PID update.

  Computes P + I + lowpass-D trim, accumulates onto input_override,
  clamps to [0, 2047×10000], and returns the current throttle output
  in [48, 2047].

  Clears integral during startup (zero_crosses < 100).

  The return value is suitable for assignment to the global `input`
  variable when called from a periodic context (e.g. 1kHz tick).
 */
uint16_t SC_Update(SpeedController *sc,
                   int      actual_e_com_time,
                   uint8_t  running,
                   uint32_t zero_crosses);

/*
  Get the throttle output.

  Returns a value in [48, 2047] suitable for the global `input` variable.
  Useful when the output must be read outside the periodic update path
  (e.g. from event-driven setInput()).
 */
uint16_t SC_GetOutput(const SpeedController *sc);

/*
  Reset PID error state.

  Called on deadband entry (adjusted_input < 47) or RPM=0 stop.
  Zeros error, integral, derivative, input_override, and lowpass state.
 */
void SC_Reset(SpeedController *sc);

/*
  Read the current target commutation interval (µs).
  Useful for diagnostics or telemetry.
 */
uint16_t SC_GetTarget(const SpeedController *sc);

#ifdef __cplusplus
}
#endif
