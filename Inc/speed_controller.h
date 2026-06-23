/*
 * speed_controller.h -- FF + I-P speed controller for AM32
 *
 * Architecture:
 *   1. Feedforward: k1*RPM + k2*RPM^2 (physical model)
 *   2. Integral accumulates target-actual error
 *   3. Proportional on measured value only (I-P form)
 *   4. Derivative on lowpass of measured value
 *   5. Observer: alpha=1/4 lowpass + linear extrapolation
 */

#pragma once

#include "targets.h"
#include <stdint.h>

#ifndef MCU_GDE23

#ifdef __cplusplus
extern "C" {
#endif

/* Default parameters */
#define SC_DEFAULT_KDRAG 4640
#define SC_DEFAULT_ADAPT_LOW 2500
#define SC_DEFAULT_ADAPT_HIGH 6000
#define SC_DEFAULT_KP 8
#define SC_DEFAULT_KI 3
#define SC_DEFAULT_KD 60

/* k1/k2 shift values */
#define SC_K1_SHIFT 10
#define SC_K2_SHIFT 28

/* Safety: shift < 32 to avoid UB */
_Static_assert(SC_K1_SHIFT < 32 && SC_K2_SHIFT < 32,
    "SC_K1_SHIFT and SC_K2_SHIFT must be < 32");

typedef struct {
    uint16_t Kp, Ki, Kd;
    int32_t error, integral, integral_limit;
    int32_t filtered_value, last_filtered, derivative;
    int32_t output_limit, pid_output;
} SC_PID;

typedef struct {
    SC_PID pid;
    uint16_t target_e_com_time;
    uint16_t k_drag;
    uint16_t adapt_low_ecom, adapt_high_ecom;
    int32_t k1, k2;
    uint16_t coeff_vbat, coeff_k_drag, coeff_kv;
    int32_t ff_compensated, input_override;
    int32_t observed_e_com_time, trend;
    uint8_t obs_init, step_count;
    int32_t last_step_interval;
    uint8_t coeff_dirty;
} SpeedController;

void SC_Init(SpeedController* sc);
void SC_SetTarget(SpeedController* sc, uint16_t e_com);
void SC_RestoreFromEeprom(SpeedController* sc);
uint16_t SC_Update(SpeedController* sc, int ecom, int ci,
    uint8_t run, uint32_t zc, uint16_t vbat);
uint16_t SC_GetOutput(const SpeedController* sc);
void SC_Reset(SpeedController* sc);
void SC_MarkCoeffDirty(SpeedController* sc);

#ifdef __cplusplus
}
#endif

#endif /* !MCU_GDE23 */
