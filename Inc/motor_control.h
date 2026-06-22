#pragma once

#include <stdint.h>
#include <stdbool.h>

void motor_start(int16_t duty);
void motor_set_duty(int16_t duty);
void motor_coast(void);
void motor_full_brake(void);
void motor_prop_brake(int16_t duty);
void motor_active_brake(void);
bool motor_is_running(void);
