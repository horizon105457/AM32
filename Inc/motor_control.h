#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
  Motor control primitives -- bottom execution layer.

  This module encapsulates all hardware-level motor operations into a closed
  set of safe primitives. The middle layer (1kHz control loop, to be built)
  composes these to implement speed control, direction logic, startup/restart,
  ramp and clamping.

  Layering:
    Upper  : DroneCANInput (ISR assigns) / signed target (rpm, duty_setpoint)
    Middle : 1kHz state machines (raw_control_loop, rpm_control_loop, ...) -- TBD
    Lower  : THIS module -- atomic hardware operations + physical safety

  Safety contract (guaranteed by this layer):
    - Critical section around comStep (blocks DShot DMA interrupt)
    - Physical clamping: |duty| <= 2000, step in [1,6], forward in {0,1}
    - motor_start is atomic (allOff + commutate + state + enableComp in lock)

  Not this layer's responsibility:
    - armed check (system-level, middle layer)
    - speed loop, FF/PID (middle layer)
    - direction-change protocol (brake-wait-reverse, middle layer)
    - stall/restart logic (middle layer)
    - ramp, startup clamp, current limit, stall protection (middle layer)

  forward contract:
    `forward` is an open-loop direction state held as a global (char forward).
    motor_set_forward() sets it; motor_commutate() uses it to pick step++/step--.
    The middle layer must set forward to match the desired physical direction
    at the appropriate time (typically before motor_start or at low-speed
    reversal). The bottom layer does NOT verify forward against actual rotation.
*/

/* === Commutation direction (open-loop state) === */
void motor_set_forward(char dir); /* dir: 1 = forward sequence (step++), 0 = reverse (step--) */
char motor_get_forward(void);

/* === Commutation stepping === */
void motor_commutate(void); /* advance one commutation step per `forward`, critical-section safe */
void motor_comstep(uint8_t step); /* set absolute step position (1-6), for startup alignment */

/* === Phase output modes === */
void motor_all_off(void); /* all phases floating (coast) */
void motor_full_brake(void); /* all low-side on (short brake) */
void motor_prop_brake(int16_t duty); /* low-side PWM proportional brake, duty in [-2000, 0] */
void motor_active_brake(void); /* comStep(2) active brake (brake_on_stop == 2) */

/* === PWM application (signed duty -> hardware) === */
/*
  duty > 0 : drive   (high-side PWM, magnitude = throttle)
  duty < 0 : brake   (low-side PWM, magnitude = brake force)
  duty = 0 : coast   (or active brake when eepromBuffer.brake_on_stop == 2)
  Rotation direction is carried by `forward` global (commutation sequence),
  NOT by the duty sign. |duty| clamped to [0, 2000].
*/
void motor_apply_duty(int16_t duty);

/* === BEMF commutation sensing (comparator interrupt) === */
void motor_enable_comparator(void); /* enable zero-cross detection for sensorless commutation */
void motor_disable_comparator(void); /* mask zero-cross interrupt (open-loop / startup) */

/* === Ignition (full start sequence, atomic) === */
/*
  Performs: allOff + commutate + commutation_interval reset + running=1 +
  enableCompInterrupts. Caller supplies initial duty via motor_apply_duty()
  afterwards (or pass duty=0 to just spin up the commutation sequence).
  duty argument is retained for API compatibility; it sets duty_cycle_setpoint.
*/
void motor_start(int16_t duty);

/* === State === */
bool motor_is_running(void);

/* === Compatibility (may be deprecated when middle layer is rewritten) === */
void motor_set_duty(int16_t duty); /* set duty_cycle_setpoint only */
void motor_coast(void); /* all_off + running=0 + duty=0 */
