/*
 * speed_controller.h -- Speed controller abstraction for AM32
 *
 * Architecture (FF + I-P):
 *   1. Feedforward -- physical model with pre-computed coefficients:
 *        k1 = 8192/(KV*Vbat)  scaled x1024    (recomputed on V change)
 *        k2 = K_DRAG/Vbat      scaled x2^28    (recomputed on V change)
 *        ff = (k1*RPM)>>10 + (k2*RPM^2)>>28   (all int32 mul + shift)
 *        ff = (thr_raw + 8192) * 12495 / 10    (final scaling, int32)
 *      KV is MOTOR_KV DroneCAN param. Vbat is real-time. K_DRAG is SPEED_FF_KDRAG.
 *   2. Integral accumulates target-actual error.
 *   3. Proportional on measured value only (I-P form).
 *   4. Derivative on lowpass of measured value.
 *   5. Observer blends raw e_com + linear-extrapolation.
 */

#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SC_D_NOISE_FLOOR  10
#define SC_OBSERVER_SHIFT 2

#define SC_DEFAULT_KDRAG            4640
#define SC_DEFAULT_ADAPT_LOW        2500
#define SC_DEFAULT_ADAPT_HIGH       6000
#define SC_DEFAULT_KP                 8
#define SC_DEFAULT_KI                 3
#define SC_DEFAULT_KD                60

/* k1 scale = 1<<10, k2 scale = 1<<28 */
#define SC_K1_SHIFT                 10
#define SC_K2_SHIFT                 28

typedef struct {
    uint16_t Kp, Ki, Kd;
    int32_t error, integral, integral_limit;
    int32_t filtered_value, last_filtered, derivative;
    int32_t output_limit, pid_output;
} SC_PID;

typedef struct {
    SC_PID pid;
    uint16_t target_e_com_time;
    uint16_t k_drag;            /* DroneCAN x1e6                      */
    uint16_t adapt_low_ecom;
    uint16_t adapt_high_ecom;
    int32_t k1;                 /* 8192/(KV*Vbat) x (1<<10)           */
    int32_t k2;                 /* K_DRAG/Vbat    x (1<<28)           */
    uint16_t coeff_vbat;        /* Vbat when k1/k2 were last computed */
    uint16_t coeff_k_drag;      /* k_drag when k2 was last computed   */
    uint16_t coeff_kv;          /* motor_kv when k1 was last computed */
    int32_t ff_baseline;
    int32_t ff_compensated;
    int32_t input_override;
    int32_t observed_e_com_time;
    int32_t trend;
    uint8_t obs_init;
    uint8_t step_count;
    int32_t last_step_interval;
} SpeedController;

void SC_Init(SpeedController *sc);
void SC_SetTarget(SpeedController *sc, uint16_t e_com);
void SC_RestoreFromEeprom(SpeedController *sc);
uint16_t SC_Update(SpeedController *sc, int ecom, int ci, uint8_t run, uint32_t zc);
uint16_t SC_GetOutput(const SpeedController *sc);
void     SC_Reset(SpeedController *sc);
uint16_t SC_GetTarget(const SpeedController *sc);
int32_t  SC_GetObserved(const SpeedController *sc);

#ifdef __cplusplus
}
#endif
