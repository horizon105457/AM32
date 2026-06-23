#pragma once

#include "eeprom.h"
#include "motor_control.h"
#include "speed_controller.h"
#include "targets.h"
#include <stdint.h>

extern EEprom_t eepromBuffer;
extern uint32_t eeprom_address;
extern uint16_t TIMER1_MAX_ARR;
extern uint16_t dshot_frametime_high;
extern uint16_t dshot_frametime_low;
extern uint32_t gcr[37];
extern uint8_t buffer_padding;
extern uint16_t adjusted_input;
extern uint32_t dma_buffer[64];
extern uint8_t dshotcommand;
extern uint16_t armed_count_threshold;
extern char forward;
extern uint8_t running;
extern uint16_t zero_input_count;
extern uint16_t signaltimeout;
extern uint16_t input;
extern volatile uint16_t newinput;
extern char play_tone_flag;
extern uint32_t current_GPIO_PIN;
extern char ic_timer_prescaler;
extern uint8_t buffersize;
extern char output_timer_prescaler;
extern volatile uint8_t compute_dshot_flag;
extern uint16_t battery_voltage;
extern int16_t actual_current;
extern uint16_t e_rpm;
extern volatile uint32_t average_interval;
extern volatile int16_t degrees_celsius;

#ifdef STMICRO
extern GPIO_TypeDef* current_GPIO_PORT;
#if !defined(MCU_F031) && !defined(MCU_G031)
extern COMP_TypeDef* active_COMP;
#endif
#endif
#ifdef GIGADEVICES
extern uint32_t current_GPIO_PORT;
#endif
#ifdef ARTERY
extern uint32_t current_GPIO_PORT;
#endif
extern uint32_t current_EXTI_LINE;
extern int e_com_time;
extern char EDT_ARM_ENABLE;
extern char EDT_ARMED;
extern uint16_t send_extended_dshot;
#if defined(NEED_INPUT_READY) || defined(NXP)
extern volatile char input_ready;
#endif
// typedef struct PID{
//	float error;
//	float Kp;
//	float Ki;
//	float Kd;
//	float integral;
//	float derivative;
//	float last_error;
//	float pid_output;
//	int16_t integral_limit;
//	int16_t output_limit;
// }PID;

typedef struct fastPID {
    int32_t error;
    uint32_t Kp;
    uint32_t Ki;
    uint32_t Kd;
    int32_t integral;
    int32_t derivative;
    int32_t last_error;
    int32_t pid_output;
    int32_t integral_limit;
    int32_t output_limit;
} fastPID;

/*
  input signal types
 */
enum inputType {
    AUTO_IN = 0,
    DSHOT_IN = 1,
    SERVO_IN = 2,
    SERIAL_IN = 3,
    EDTARM_IN = 4,
    DRONECAN_IN = 5,
};

/*
  DroneCAN input interface
  ISR: decode + validate + assign, no control logic
  1kHz loop: routing + scaling + arming + direction
 */
#ifndef RPM_SHIFT
#define RPM_SHIFT 0
#endif

#define DRONECAN_MODE_NONE 0
#define DRONECAN_MODE_RAW 1
#define DRONECAN_MODE_RPM 2

struct DroneCANInput {
    volatile int16_t raw_throttle; // -8192~8191 (RAW mode)
    volatile int16_t rpm_target; // RPM value (compressed by RPM_SHIFT)
    volatile uint8_t mode; // DRONECAN_MODE_NONE / RAW / RPM
    volatile bool ready; // new data pending
};

extern struct DroneCANInput dronecan_input;
extern bool dronecan_armed;

#ifndef MCU_GDE23
/* RPM closed-loop control */
extern SpeedController sc;
extern volatile int32_t newrpm;
extern volatile uint8_t rpmSet;
extern int32_t rpm;
extern uint8_t rpm_active;
extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
extern uint32_t MINIMUM_RPM_SPEED_CONTROL;
#endif

/* Motor control interface state */
extern int16_t duty_cycle_setpoint;
extern int16_t last_duty_cycle;
extern uint16_t min_startup_duty;

/* Bottom-layer hardware state (owned by main.c, accessed by motor_control.c) */
extern uint16_t tim1_arr;
extern uint16_t adjusted_duty_cycle;
extern char step;
extern char prop_brake_active;
extern volatile uint32_t commutation_interval;
extern volatile char armed;
