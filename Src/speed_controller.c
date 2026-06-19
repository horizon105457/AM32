/*
 * speed_controller.c -- FF + I-P speed controller (optimized)
 *
 * feedforward = k1*RPM + k2*RPM^2  with pre-computed scaled coefficients.
 * k1, k2 recomputed only when Vbat or KV changes.
 *
 * Sign convention for e_com domain:
 *   e_com UP = motor slower = need MORE output
 *   So:  error  = observed - target  (positive = need more throttle)
 *        I = +Ki * integral(error)
 *        P = +Kp * y               (positive: damping in phase with need)
 *        D = +Kd * dy              (positive: oppose deceleration)
 */

#include "speed_controller.h"
#include "eeprom.h"

#define ECOM_PHYS_MIN  100
#define ECOM_PHYS_MAX  20000

extern uint16_t battery_voltage;
extern uint16_t motor_kv;

/* -- recompute k1, k2 when voltage or KV changes -------------------- */
static void update_coeffs(SpeedController *sc)
{
    uint16_t vb = battery_voltage;   /* x10, e.g. 210 = 21.0 V */
    if (vb < 60) vb = 60;
    uint16_t kv = motor_kv;
    if (kv < 20) kv = 20;

    /* vb is x10, so physical Vbat = vb/10.  Compensate with x10 in num. */
    sc->k1 = (int32_t)(((uint64_t)8192 * 10 << SC_K1_SHIFT) / ((uint64_t)kv * vb));
    sc->k2 = (int32_t)(((uint64_t)sc->k_drag * 10ULL << SC_K2_SHIFT) / (1000000ULL * vb));
    sc->coeff_vbat = vb;
    sc->coeff_k_drag = sc->k_drag;
    sc->coeff_kv = motor_kv;
}

/* -- feedforward: thr = (k1*RPM)>>K1 + (k2*RPM^2)>>K2, then to ff -- */
static int32_t compute_ff(const SpeedController *sc, uint16_t target_ecom)
{
    uint16_t pp = (uint16_t)eepromBuffer.motor_poles / 2;
    if (pp < 1) pp = 1;
    uint32_t rpm = 60000000UL / pp / target_ecom;

    /* thr_raw = BEMF + drag (all int64-safe, >> after mul) */
    int32_t thr = (int32_t)(((int64_t)sc->k1 * rpm) >> SC_K1_SHIFT);
    thr       += (int32_t)(((int64_t)sc->k2 * rpm * rpm) >> SC_K2_SHIFT);
    if (thr < 0) thr = 0;

    /* ff(x10000) = (thr+8192) * 20470000/16383 = (thr+8192)*12495/10 */
    int32_t ff = (int32_t)(((int64_t)(thr + 8192) * 12495) / 10);
    if (ff < 48 * 10000) ff = 48 * 10000;
    if (ff > 2047 * 10000) ff = 2047 * 10000;
    return ff;
}

/* -- adaptive gain scale (x256) ------------------------------------- */
static int32_t ags(const SpeedController *sc, int32_t obs)
{
    uint16_t lo = sc->adapt_low_ecom, hi = sc->adapt_high_ecom;
    if (lo < 100) lo = SC_DEFAULT_ADAPT_LOW;
    if (hi <= lo) hi = lo + 100;
    if (obs <= (int32_t)lo) return 154;
    if (obs >= (int32_t)hi) return 384;
    return 154 + (obs - (int32_t)lo) * (384 - 154) / (int32_t)(hi - lo);
}

/* -- observer (alpha=1/4 lowpass + trend) --------------------------- */
static void obs(SpeedController *sc, int32_t raw, int fresh)
{
    if (!sc->obs_init) {
        sc->observed_e_com_time = raw;
        sc->trend = 0;
        sc->obs_init = 1;
        return;
    }
    if (fresh) {
        int32_t pred = sc->observed_e_com_time + sc->trend;
        sc->observed_e_com_time = pred + ((raw - pred) >> SC_OBSERVER_SHIFT);
        sc->trend += (sc->observed_e_com_time - pred - sc->trend) >> SC_OBSERVER_SHIFT;
    } else {
        sc->observed_e_com_time += sc->trend;
        sc->trend -= sc->trend >> 4;
    }
    if (sc->observed_e_com_time < ECOM_PHYS_MIN) sc->observed_e_com_time = ECOM_PHYS_MIN;
    if (sc->observed_e_com_time > ECOM_PHYS_MAX) sc->observed_e_com_time = ECOM_PHYS_MAX;
}

/* -- public ---------------------------------------------------------- */
void SC_Init(SpeedController *sc)
{
    *sc = (SpeedController){0};
    sc->k_drag = SC_DEFAULT_KDRAG;
    sc->adapt_low_ecom  = SC_DEFAULT_ADAPT_LOW;
    sc->adapt_high_ecom = SC_DEFAULT_ADAPT_HIGH;
    sc->pid = (SC_PID){
        .Kp = SC_DEFAULT_KP, .Ki = SC_DEFAULT_KI, .Kd = SC_DEFAULT_KD,
        .integral_limit = 500000, .output_limit = 500000,
    };
    update_coeffs(sc);  /* k1,k2 valid before first SetTarget */
}

void SC_SetTarget(SpeedController *sc, uint16_t ecom)
{
    sc->target_e_com_time = ecom;
    sc->ff_baseline    = compute_ff(sc, ecom);
    sc->ff_compensated = sc->ff_baseline;
    sc->input_override = sc->ff_baseline;
    sc->obs_init = 0;
}

/*
 * Restore speed PID parameters from eepromBuffer.spd.* (v4+).
 * Must be called after SC_Init and loadEEpromSettings.
 */
void SC_RestoreFromEeprom(SpeedController *sc)
{
    sc->pid.Kp          = eepromBuffer.spd.speed_kp;
    sc->pid.Ki          = eepromBuffer.spd.speed_ki;
    sc->pid.Kd          = eepromBuffer.spd.speed_kd;
    sc->k_drag          = eepromBuffer.spd.speed_ff_kdrag;
    sc->adapt_low_ecom  = (uint16_t)eepromBuffer.spd.speed_adapt_lo * 100;
    sc->adapt_high_ecom = (uint16_t)eepromBuffer.spd.speed_adapt_hi * 100;
    update_coeffs(sc);  /* k_drag may have changed → refresh k2 */
}

uint16_t SC_Update(SpeedController *sc, int ecom, int ci, uint8_t run, uint32_t zc)
{
    if (!run) return SC_GetOutput(sc);
    SC_PID *p = &sc->pid;

    /* -- per-step freshness & observer -- */
    int fresh = 0; int32_t se = 0;
    if (ci != sc->last_step_interval) {
        sc->last_step_interval = ci;
        if (++sc->step_count >= 6) { sc->step_count = 0; fresh = 1; }
        se = (int32_t)ci * 6;
    }
    if (fresh) obs(sc, ecom, 1);
    else if (se > 0) obs(sc, se, 1);
    else obs(sc, 0, 0);

    int32_t o  = sc->observed_e_com_time;
    int32_t gs = ags(sc, o);

    /*
     * Error in e_com domain:  positive = motor too slow = need more throttle.
     * P and D signs are POSITIVE for e_com (inverse of standard I-P).
     */
    int32_t err = o - (int32_t)sc->target_e_com_time;  /* positive -> speed up */

    /* Lowpass filter for D */
    p->filtered_value += (o - p->filtered_value) >> 3;

    /* Proportional: +Kp*y  (positive: damping in phase with need) */
    int32_t kp = (int32_t)p->Kp * gs / 256;
    int32_t pt = (kp * p->filtered_value / 100);

    /* Derivative: +Kd*dy  (positive: oppose deceleration) */
    p->derivative = 0;
    int32_t df = p->filtered_value - p->last_filtered;
    p->last_filtered = p->filtered_value;
    if (df > SC_D_NOISE_FLOOR || df < -SC_D_NOISE_FLOOR) {
        int32_t kd = (int32_t)p->Kd * gs / 256;
        p->derivative = (kd * df / 100);
    }

    /* Coefficient refresh on Vbat, k_drag, or motor_kv change */
    if (battery_voltage != sc->coeff_vbat || sc->k_drag != sc->coeff_k_drag || motor_kv != sc->coeff_kv) {
        update_coeffs(sc);
        sc->ff_compensated = compute_ff(sc, sc->target_e_com_time);
    }

    /* Integral with clamping anti-windup (P+D computed first) */
    if (zc < 100) {
        p->integral = 0;
    } else {
        int32_t ki = (int32_t)p->Ki * gs / 256;
        int32_t di = err * ki / 100;
        int32_t cand = sc->ff_compensated + p->integral + di + pt + p->derivative;
        if (!((cand > 2047 * 10000 && di > 0) || (cand < 0 && di < 0))) {
            p->integral += di;
        }
        if (p->integral > p->integral_limit)  p->integral = p->integral_limit;
        if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;
    }

    int32_t trim = p->integral + pt + p->derivative;
    if (trim > p->output_limit)  trim = p->output_limit;
    if (trim < -p->output_limit) trim = -p->output_limit;

    sc->input_override = sc->ff_compensated + trim;
    if (sc->input_override > 2047 * 10000) sc->input_override = 2047 * 10000;
    if (sc->input_override < 0)            sc->input_override = 0;
    p->error = err; p->pid_output = trim;
    return SC_GetOutput(sc);
}

uint16_t SC_GetOutput(const SpeedController *sc) {
    uint16_t t = (uint16_t)(sc->input_override / 10000);
    if (t < 48) t = 48;
    if (t > 2047) t = 2047;
    return t;
}

void SC_Reset(SpeedController *sc) {
    sc->pid.error = sc->pid.integral = sc->pid.derivative = 0;
    sc->pid.filtered_value = sc->pid.last_filtered = sc->pid.pid_output = 0;
    sc->input_override = 0; sc->obs_init = 0; sc->trend = 0;
    sc->step_count = 0; sc->last_step_interval = 0;
}

uint16_t SC_GetTarget(const SpeedController *sc)   { return sc->target_e_com_time; }
int32_t  SC_GetObserved(const SpeedController *sc) { return sc->observed_e_com_time; }
