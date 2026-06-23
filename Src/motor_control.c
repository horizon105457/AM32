#include "motor_control.h"

#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

extern void commutate(void);

/* === Commutation direction (open-loop state) === */

void motor_set_forward(char dir)
{
    forward = dir ? 1 : 0;
}

char motor_get_forward(void)
{
    return forward;
}

/* === Commutation stepping === */

void motor_commutate(void)
{
    commutate();
}

void motor_comstep(uint8_t s)
{
    if (s < 1)
        s = 1;
    if (s > 6)
        s = 6;
    __disable_irq();
    comStep(s);
    __enable_irq();
}

/* === Phase output modes === */

void motor_all_off(void)
{
    allOff();
}

void motor_full_brake(void)
{
    fullBrake();
    running = 0;
    duty_cycle_setpoint = 0;
}

void motor_prop_brake(int16_t duty)
{
    if (duty < -2000)
        duty = -2000;
    if (duty > 0)
        duty = 0;
    proportionalBrake();
    duty_cycle_setpoint = duty;
}

void motor_active_brake(void)
{
    comStep(2);
}

/* === PWM application (signed duty -> hardware) === */

void motor_apply_duty(int16_t duty)
{
    if (duty > 2000)
        duty = 2000;
    if (duty < -2000)
        duty = -2000;

    if (duty > 0) {
        adjusted_duty_cycle = ((duty * tim1_arr) / 2000) + 1;
    } else if (duty < 0) {
        adjusted_duty_cycle = tim1_arr - (((uint16_t)(-duty) * tim1_arr) / 2000);
    } else {
        if ((eepromBuffer.brake_on_stop == 2) && armed) {
            motor_active_brake();
            adjusted_duty_cycle = DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000) * 10;
        } else {
            adjusted_duty_cycle = 0;
        }
    }
}

/* === BEMF commutation sensing === */

void motor_enable_comparator(void)
{
    enableCompInterrupts();
}

void motor_disable_comparator(void)
{
    maskPhaseInterrupts();
}

/* === Ignition (full start sequence) === */

void motor_start(int16_t duty)
{
    allOff();
    commutate();
    commutation_interval = 10000;
    SET_INTERVAL_TIMER_COUNT(5000);
    running = 1;
    enableCompInterrupts();
    last_duty_cycle = min_startup_duty;
    duty_cycle_setpoint = duty;
}

/* === State === */

bool motor_is_running(void)
{
    return running != 0;
}

/* === Compatibility === */

void motor_set_duty(int16_t duty)
{
    duty_cycle_setpoint = duty;
}

void motor_coast(void)
{
    allOff();
    running = 0;
    duty_cycle_setpoint = 0;
}
