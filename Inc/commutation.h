#ifndef COMMUTATION_H_
#define COMMUTATION_H_

#include "targets.h"
#include "peripherals.h"

/* INTERVAL and COM have matching ticks on these hardware groups.
 * E230 retains its existing polling implementation to fit its flash limit.
 * F415, NXP, WCH and SITL also retain their existing implementation. */
#if (defined(STMICRO) && !defined(MCU_SITL)) || defined(MCU_AT421)
#define ASYNC_POLLING_COMMUTATION

/* Only normal crossings enter the estimator. Timeout recovery is separate. */
static inline uint8_t commutationCaptureValid(uint32_t period, uint8_t overflow)
{
    return !overflow && period != 0 && period <= 45000;
}

/* The common 16-bit interval bounds the product and delay before narrowing.
 * delay <= 32767 also proves the COMP reload delay+1 fits uint16_t. */
static inline uint8_t calculateCommutationTiming(uint32_t interval,
    uint32_t advance_level, uint32_t* new_advance, uint16_t* delay)
{
    if (interval == 0 || interval > UINT16_MAX || advance_level > 32) {
        return 0;
    }
    uint32_t timing_advance = (interval * advance_level) >> 6;
    uint32_t half_period = interval >> 1;
    if (timing_advance > half_period) {
        return 0;
    }
    *new_advance = timing_advance;
    *delay = (uint16_t)(half_period - timing_advance);
    return 1;
}

/* Called within a short transaction preserving PRIMASK. */
static inline void stopCommutationTimer(void)
{
    DISABLE_COM_TIMER_INT();
#if defined(STMICRO)
    COM_TIMER->CR1 &= ~TIM_CR1_CEN;
    COM_TIMER->SR = 0;
#else
    COM_TIMER->ctrl1_bit.tmren = FALSE;
    COM_TIMER->ists = 0;
#endif
}

static inline void loadCommutationTimer(uint16_t reload)
{
    stopCommutationTimer();
#if defined(STMICRO)
    COM_TIMER->ARR = reload;
    COM_TIMER->EGR = TIM_EGR_UG;
    COM_TIMER->SR = 0;
#else
    COM_TIMER->pr = reload;
    COM_TIMER->swevt = TMR_OVERFLOW_SWTRIG;
    COM_TIMER->ists = 0;
#endif
}

static inline void startCommutationTimer(uint16_t elapsed)
{
#if defined(STMICRO)
    COM_TIMER->CNT = elapsed;
    ENABLE_COM_TIMER_INT();
    COM_TIMER->CR1 |= TIM_CR1_CEN;
#else
    COM_TIMER->cval = elapsed;
    ENABLE_COM_TIMER_INT();
    COM_TIMER->ctrl1_bit.tmren = TRUE;
#endif
}

static inline void clearCommutationIntervalOverflow(void)
{
#if defined(STMICRO)
    INTERVAL_TIMER->SR = ~TIM_SR_UIF;
#else
    INTERVAL_TIMER->ists = (uint16_t)~TMR_OVF_FLAG;
#endif
}

static inline uint8_t commutationIntervalOverflowed(void)
{
#if defined(STMICRO)
    return (INTERVAL_TIMER->SR & TIM_SR_UIF) != 0;
#else
    return (INTERVAL_TIMER->ists & TMR_OVF_FLAG) != 0;
#endif
}

static inline void resetCommutationInterval(uint16_t count)
{
    SET_INTERVAL_TIMER_COUNT(count);
    clearCommutationIntervalOverflow();
}
#endif

void cancelCommutation(void);

#endif
