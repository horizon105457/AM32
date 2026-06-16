/*
 * speed_controller.c — Speed controller implementation
 *
 * Architecture: feedforward baseline + P + I + lowpass-D trim.
 *
 *   output = ff_baseline  +  (P · error  +  I · ∫error  +  D · Δfiltered_error)
 *
 * The feedforward maps target_e_com_time → baseline throttle (open-loop),
 * so the PID only needs to micro-trim around that baseline.  This
 * eliminates the saturation problem of the previous accumulate-from-zero
 * design and greatly reduces startup settling time.
 *
 * The controller operates in the commutation-interval domain (µs).
 * RPM ↔ µs conversion is the caller's responsibility.
 */

#include "speed_controller.h"

// constants for the first-order error lowpass (power-of-2 shift)
#define D_LOWPASS_SHIFT 3         // α = 1/8, cutoff ≈ 20 Hz at 1 kHz sample rate
#define FF_ECOM_MIN     200       // µs  (fastest expected commutation interval)
#define FF_ECOM_MAX     10000     // µs  (slowest expected commutation interval)

/*
  Default PID gains (runtime-overridable via DroneCAN param Set
  pointing directly into sc.pid.Kp / sc.pid.Ki / sc.pid.Kd).
 */
#define DEFAULT_SPEED_KP 5
#define DEFAULT_SPEED_KI 1
#define DEFAULT_SPEED_KD 50

/* ----------------------------------------------------------------- */
/*  Feedforward — linear interpolation in the e_com_time domain      */
/*                                                                   */
/*  target_e_com_time 短 → RPM 高 → 需要大油门                       */
/*  target_e_com_time 长 → RPM 低 → 需要小油门                       */
/*                                                                   */
/*  返回 scale ×10000 的值, range: 48×10000 ~ 2047×10000            */
/* ----------------------------------------------------------------- */
static int32_t compute_feedforward(uint16_t target_e_com_time)
{
    if (target_e_com_time <= FF_ECOM_MIN) {
        return 2047 * 10000;
    }
    if (target_e_com_time >= FF_ECOM_MAX) {
        return 48 * 10000;
    }

    /*
      Linear interpolation:  e_com 200 → full,  e_com 10000 → idle
        ff = 2047*10000 - (target-200) * 1999*10000 / (10000-200)

      Simplify to avoid 64-bit:  1999*10000/9800 = 19990000/9800 = 99950/49
      Max intermediate: 9800 * 99950 = 979,510,000 < INT32_MAX ✓
    */
    int32_t t = 2047 * 10000 - (int32_t)(target_e_com_time - FF_ECOM_MIN) * 99950 / 49;
    if (t < 48 * 10000)  t = 48 * 10000;
    if (t > 2047 * 10000) t = 2047 * 10000;
    return t;
}

/* ----------------------------------------------------------------- */
/*  Init — reads PID gains from globals for runtime configurability   */
/* ----------------------------------------------------------------- */
void SC_Init(SpeedController *sc)
{
    sc->target_e_com_time   = 0;
    sc->ff_baseline         = 0;
    sc->input_override      = 0;
    sc->filtered_error      = 0;
    sc->last_filtered_error = 0;

    sc->pid = (fastPID){
        .Kp             = DEFAULT_SPEED_KP,
        .Ki             = DEFAULT_SPEED_KI,
        .Kd             = DEFAULT_SPEED_KD,
        .integral_limit = 10000,
        .output_limit   = 50000,
        .error          = 0,
        .integral       = 0,
        .derivative     = 0,
        .last_error     = 0,
        .pid_output     = 0,
    };
}

/* ----------------------------------------------------------------- */
/*  Set target and recompute feedforward baseline                    */
/* ----------------------------------------------------------------- */
void SC_SetTarget(SpeedController *sc, uint16_t target_e_com_time)
{
    sc->target_e_com_time = target_e_com_time;
    sc->ff_baseline = compute_feedforward(target_e_com_time);
    /*
      Re-anchor the accumulator to the new feedforward so that
      a target change doesn't leave the PID chasing a stale baseline.
     */
    sc->input_override = sc->ff_baseline;
}

/* ----------------------------------------------------------------- */
/*  Set feedforward directly (advanced callers)                      */
/* ----------------------------------------------------------------- */
void SC_SetFeedforward(SpeedController *sc, int32_t ff_baseline)
{
    sc->ff_baseline = ff_baseline;
}

/* ----------------------------------------------------------------- */
/*  1kHz PID update                                                   */
/* ----------------------------------------------------------------- */
uint16_t SC_Update(SpeedController *sc,
                   int      actual_e_com_time,
                   uint8_t  running,
                   uint32_t zero_crosses)
{
    if (!running) {
        return SC_GetOutput(sc);
    }

    fastPID *pid = &sc->pid;

    /* ── Error ──────────────────────────────────────────────── */
    int32_t error = actual_e_com_time - (int)sc->target_e_com_time;

    /* ── First-order lowpass on error (for D term only) ──────── */
    sc->filtered_error += (error - sc->filtered_error) >> D_LOWPASS_SHIFT;

    /* ── Proportional ────────────────────────────────────────── */
    int32_t p_term = error * (int32_t)pid->Kp;

    /* ── Integral (cleared during startup) ───────────────────── */
    if (zero_crosses < 100) {
        pid->integral = 0;
    } else {
        pid->integral += error * (int32_t)pid->Ki;
        if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

    /* ── Derivative (on lowpass error, with noise floor) ─────── */
    pid->derivative = 0;
    int32_t df = sc->filtered_error - sc->last_filtered_error;
    sc->last_filtered_error = sc->filtered_error;
    if (df > SC_D_NOISE_FLOOR || df < -SC_D_NOISE_FLOOR) {
        pid->derivative = pid->Kd * df;
    }

    /* ── PID trim ────────────────────────────────────────────── */
    int32_t pid_trim = p_term + pid->integral + pid->derivative;
    if (pid_trim > pid->output_limit)  pid_trim = pid->output_limit;
    if (pid_trim < -pid->output_limit) pid_trim = -pid->output_limit;

    /* ── Output = feedforward + pid_trim ─────────────────────── */
    sc->input_override = sc->ff_baseline + pid_trim;

    /* ── Clamp ───────────────────────────────────────────────── */
    if (sc->input_override > 2047 * 10000)  sc->input_override = 2047 * 10000;
    if (sc->input_override < 0)             sc->input_override = 0;

    /* bookkeeping */
    pid->error = error;
    pid->last_error = error;
    pid->pid_output = pid_trim;

    return SC_GetOutput(sc);
}

/* ----------------------------------------------------------------- */
/*  Get throttle output                                               */
/* ----------------------------------------------------------------- */
uint16_t SC_GetOutput(const SpeedController *sc)
{
    uint16_t t = (uint16_t)(sc->input_override / 10000);
    if (t < 48)  t = 48;
    if (t > 2047) t = 2047;
    return t;
}

/* ----------------------------------------------------------------- */
/*  Reset                                                             */
/* ----------------------------------------------------------------- */
void SC_Reset(SpeedController *sc)
{
    sc->pid.error      = 0;
    sc->pid.integral   = 0;
    sc->pid.derivative = 0;
    sc->pid.last_error = 0;
    sc->pid.pid_output = 0;
    sc->input_override = 0;
    sc->filtered_error = 0;
    sc->last_filtered_error = 0;
}

/* ----------------------------------------------------------------- */
/*  Get target                                                        */
/* ----------------------------------------------------------------- */
uint16_t SC_GetTarget(const SpeedController *sc)
{
    return sc->target_e_com_time;
}
