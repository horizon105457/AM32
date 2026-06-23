/*
 * speed_controller.c -- FF + I-P speed controller for AM32
 *
 * Feedforward: k1*RPM + k2*RPM^2 with pre-computed coefficients.
 * k1, k2 recomputed when Vbat changes (passed as parameter).
 *
 * Sign convention for e_com domain:
 *   e_com UP = motor slower = need MORE output
 *   error = observed - target (positive = need more throttle)
 */

#include "targets.h"

#ifndef MCU_GDE23

#include "eeprom.h"
#include "speed_controller.h"

#define ECOM_PHYS_MIN 100
#define ECOM_PHYS_MAX 60000

/* Recompute k1, k2 when voltage changes */
static void update_coeffs(SpeedController* sc, uint16_t vbat)
{
    uint16_t vb = vbat;
    if (vb < 60)
        vb = 60;
    uint16_t kv = eepromBuffer.motor_kv;
    if (kv < 20)
        kv = 20;

    sc->k1 = (int32_t)(((uint64_t)8192 * 10 << SC_K1_SHIFT) / ((uint64_t)kv * vb));
    sc->k2 = (int32_t)(((uint64_t)sc->k_drag * 10ULL << SC_K2_SHIFT) / (1000000ULL * vb));
    sc->coeff_vbat = vb;
    sc->coeff_k_drag = sc->k_drag;
    sc->coeff_kv = eepromBuffer.motor_kv;
}

/* Feedforward: thr = (k1*RPM)>>K1 + (k2*RPM^2)>>K2 */
static int32_t compute_ff(const SpeedController* sc, uint16_t target_ecom,
    uint16_t poles, uint32_t max_rpm)
{
    uint16_t pp = poles / 2;
    if (pp < 1)
        pp = 1;
    uint32_t rpm = 60000000UL / pp / target_ecom;

    int32_t thr = (int32_t)(((int64_t)sc->k1 * rpm) >> SC_K1_SHIFT);
    thr += (int32_t)(((int64_t)sc->k2 * rpm * rpm) >> SC_K2_SHIFT);
    if (thr < 0)
        thr = 0;

    if (max_rpm < 1000)
        max_rpm = 1000;
    int32_t thr_max = (int32_t)(((int64_t)sc->k1 * max_rpm) >> SC_K1_SHIFT)
        + (int32_t)(((int64_t)sc->k2 * max_rpm * max_rpm) >> SC_K2_SHIFT);
    if (thr_max < 100)
        thr_max = 100;

    int32_t ff = (48 * 10000)
        + (int32_t)(((int64_t)thr * (2047 - 48) * 10000) / thr_max);
    if (ff > 2047 * 10000)
        ff = 2047 * 10000;
    return ff;
}

/* Adaptive gain scale (x256) */
static int32_t ags(const SpeedController* sc, int32_t obs)
{
    uint16_t lo = sc->adapt_low_ecom, hi = sc->adapt_high_ecom;
    if (lo < 100)
        lo = SC_DEFAULT_ADAPT_LOW;
    if (hi <= lo)
        hi = lo + 100;
    if (obs <= (int32_t)lo)
        return 154;
    if (obs >= (int32_t)hi)
        return 384;
    return 154 + (obs - (int32_t)lo) * (384 - 154) / (int32_t)(hi - lo);
}

/* Observer (alpha=1/4 lowpass + trend) */
static void obs(SpeedController* sc, int32_t raw, int fresh)
{
    if (!sc->obs_init) {
        sc->observed_e_com_time = raw;
        sc->trend = 0;
        sc->obs_init = 1;
        return;
    }
    if (fresh) {
        int32_t pred = sc->observed_e_com_time + sc->trend;
        sc->observed_e_com_time = pred + ((raw - pred) >> 2);
        sc->trend += (sc->observed_e_com_time - pred - sc->trend) >> 2;
    } else {
        sc->observed_e_com_time += sc->trend;
        sc->trend -= sc->trend >> 4;
    }
    if (sc->observed_e_com_time < ECOM_PHYS_MIN)
        sc->observed_e_com_time = ECOM_PHYS_MIN;
    if (sc->observed_e_com_time > ECOM_PHYS_MAX)
        sc->observed_e_com_time = ECOM_PHYS_MAX;
}

/* Public API */

void SC_Init(SpeedController* sc)
{
    *sc = (SpeedController) { 0 };
    sc->k_drag = SC_DEFAULT_KDRAG;
    sc->adapt_low_ecom = SC_DEFAULT_ADAPT_LOW;
    sc->adapt_high_ecom = SC_DEFAULT_ADAPT_HIGH;
    sc->pid = (SC_PID) {
        .Kp = SC_DEFAULT_KP,
        .Ki = SC_DEFAULT_KI,
        .Kd = SC_DEFAULT_KD,
        .integral_limit = 3000000,
        .output_limit = 3000000,
    };
}

void SC_SetTarget(SpeedController* sc, uint16_t ecom)
{
    sc->target_e_com_time = ecom;
    sc->observed_e_com_time = (int32_t)ecom;
    sc->obs_init = 1;
}

void SC_RestoreFromEeprom(SpeedController* sc)
{
    sc->pid.Kp = eepromBuffer.can.spd.speed_kp;
    sc->pid.Ki = eepromBuffer.can.spd.speed_ki;
    sc->pid.Kd = eepromBuffer.can.spd.speed_kd;
    sc->k_drag = eepromBuffer.can.spd.speed_ff_kdrag;
    sc->adapt_low_ecom = (uint16_t)eepromBuffer.can.spd.speed_adapt_lo * 100;
    sc->adapt_high_ecom = (uint16_t)eepromBuffer.can.spd.speed_adapt_hi * 100;
    sc->coeff_dirty = 1;
}

uint16_t SC_Update(SpeedController* sc, int ecom, int ci,
    uint8_t run, uint32_t zc, uint16_t vbat)
{
    if (!run) {
        sc->last_step_interval = ci;
        /* Compute FF on first call after SetTarget */
        if (sc->coeff_dirty || sc->coeff_vbat != vbat) {
            update_coeffs(sc, vbat);
            uint16_t poles = eepromBuffer.motor_poles;
            if (poles < 2)
                poles = 2;
            extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
            sc->ff_compensated = compute_ff(sc, sc->target_e_com_time,
                poles, MAXIMUM_RPM_SPEED_CONTROL);
            sc->coeff_dirty = 0;
        }
        sc->input_override = sc->ff_compensated;
        return SC_GetOutput(sc);
    }

    SC_PID* p = &sc->pid;

    /* Per-step freshness & observer */
    int fresh = 0;
    int32_t se = 0;
    if (ci != sc->last_step_interval) {
        sc->last_step_interval = ci;
        if (++sc->step_count >= 6) {
            sc->step_count = 0;
            fresh = 1;
        }
        se = (int32_t)ci * 6;
    }
    if (fresh)
        obs(sc, ecom, 1);
    else if (se > 0)
        obs(sc, se, 1);
    else
        obs(sc, 0, 0);

    int32_t o = sc->observed_e_com_time;
    int32_t gs = ags(sc, o);

    int32_t err = o - (int32_t)sc->target_e_com_time;

    /* Lowpass filter for D */
    p->filtered_value += (o - p->filtered_value) >> 3;

    /* Proportional: +Kp*y */
    int32_t kp = (int32_t)p->Kp * gs / 256;
    int32_t pt = (kp * p->filtered_value / 100);

    /* Derivative: +Kd*dy */
    p->derivative = 0;
    int32_t df = p->filtered_value - p->last_filtered;
    p->last_filtered = p->filtered_value;
    if (df > 10 || df < -10) {
        int32_t kd = (int32_t)p->Kd * gs / 256;
        p->derivative = (kd * df / 100);
    }

    /* Coefficient refresh */
    if (vbat != sc->coeff_vbat || sc->coeff_dirty) {
        update_coeffs(sc, vbat);
        uint16_t poles = eepromBuffer.motor_poles;
        if (poles < 2)
            poles = 2;
        extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
        sc->ff_compensated = compute_ff(sc, sc->target_e_com_time,
            poles, MAXIMUM_RPM_SPEED_CONTROL);
        sc->coeff_dirty = 0;
    }

    /* Integral with anti-windup */
    if (zc < 100) {
        p->integral = 0;
    } else {
        int32_t ki = (int32_t)p->Ki * gs / 256;
        int32_t di = err * ki / 100;
        int32_t cand = sc->ff_compensated + p->integral + di + pt + p->derivative;
        if (!((cand > 2047 * 10000 && di > 0) || (cand < 0 && di < 0))) {
            p->integral += di;
        }
        if (p->integral > p->integral_limit)
            p->integral = p->integral_limit;
        if (p->integral < -p->integral_limit)
            p->integral = -p->integral_limit;
    }

    int32_t trim = p->integral + pt + p->derivative;
    if (trim > p->output_limit)
        trim = p->output_limit;
    if (trim < -p->output_limit)
        trim = -p->output_limit;

    sc->input_override = sc->ff_compensated + trim;
    if (sc->input_override > 2047 * 10000)
        sc->input_override = 2047 * 10000;
    if (sc->input_override < 0)
        sc->input_override = 0;
    p->error = err;
    p->pid_output = trim;
    return SC_GetOutput(sc);
}

uint16_t SC_GetOutput(const SpeedController* sc)
{
    uint16_t t = (uint16_t)(sc->input_override / 10000);
    if (t < 48)
        t = 48;
    if (t > 2047)
        t = 2047;
    return t;
}

void SC_Reset(SpeedController* sc)
{
    sc->pid.error = sc->pid.integral = sc->pid.derivative = 0;
    sc->pid.filtered_value = sc->pid.last_filtered = sc->pid.pid_output = 0;
    sc->input_override = 0;
    sc->obs_init = 0;
    sc->trend = 0;
    sc->step_count = 0;
    sc->last_step_interval = 0;
    sc->coeff_dirty = 0;
}

void SC_MarkCoeffDirty(SpeedController* sc)
{
    sc->coeff_dirty = 1;
}

#endif /* !MCU_GDE23 */
