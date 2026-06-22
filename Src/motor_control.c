#include "motor_control.h"
#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

extern void commutate(void);

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

void motor_full_brake(void)
{
    fullBrake();
    running = 0;
    duty_cycle_setpoint = 0;
}

void motor_prop_brake(int16_t duty)
{
    proportionalBrake();
    duty_cycle_setpoint = duty;
}

void motor_active_brake(void)
{
    comStep(2);
}

bool motor_is_running(void)
{
    return running != 0;
}
