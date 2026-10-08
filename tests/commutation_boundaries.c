/* Host register model for the actual commutation.h helpers; not timing proof. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static struct {
    uint32_t CNT, ARR, CR1, DIER, SR, EGR;
} interval_timer, com_timer;

#define STMICRO
#define COM_TIMER (&com_timer)
#define INTERVAL_TIMER (&interval_timer)
#define TIM_CR1_CEN 1u
#define TIM_DIER_UIE 1u
#define TIM_SR_UIF 1u
#define TIM_EGR_UG 1u
#define DISABLE_COM_TIMER_INT() (COM_TIMER->DIER &= ~TIM_DIER_UIE)
#define ENABLE_COM_TIMER_INT() (COM_TIMER->DIER |= TIM_DIER_UIE)
#define INTERVAL_TIMER_COUNT (INTERVAL_TIMER->CNT)
#define SET_INTERVAL_TIMER_COUNT(v) (INTERVAL_TIMER->CNT = (v))

/* The test runner provides empty platform include stubs in its temp folder. */
#include "commutation.h"

int main(void)
{
    assert(!commutationCaptureValid(0, 0));
    assert(commutationCaptureValid(1, 0));
    assert(commutationCaptureValid(45000, 0));
    assert(!commutationCaptureValid(45001, 0));
    assert(!commutationCaptureValid(123, 1));

    uint32_t advance = 123;
    uint16_t delay = 123;
    assert(!calculateCommutationTiming(0, 16, &advance, &delay));
    assert(!calculateCommutationTiming(65536, 16, &advance, &delay));
    assert(!calculateCommutationTiming(UINT32_MAX, 16, &advance, &delay));
    assert(!calculateCommutationTiming(10000, UINT32_MAX, &advance, &delay));
    assert(advance == 123 && delay == 123); /* invalid does not publish */
    assert(calculateCommutationTiming(10000, 16, &advance, &delay));
    assert(advance == 2500 && delay == 2500);
    assert(calculateCommutationTiming(1, 32, &advance, &delay));
    assert(delay == 0);

    /* Exhaustive valid periods/advance: no narrowed wrap, COMP+1 fits, and
     * POLL-1 is only used when delay != 0. Exercise the production function. */
    for (uint32_t period = 1; period <= UINT16_MAX; period++) {
        for (uint32_t level = 0; level <= 32; level++) {
            assert(calculateCommutationTiming(period, level, &advance, &delay));
            assert(delay <= 32767);
            assert((uint32_t)delay + 1 <= UINT16_MAX);
            assert(advance + delay == (period >> 1));
            if (delay) {
                loadCommutationTimer(delay - 1);
                assert(COM_TIMER->ARR == (uint32_t)delay - 1);
            }
        }
    }

    COM_TIMER->CR1 = TIM_CR1_CEN;
    COM_TIMER->DIER = TIM_DIER_UIE;
    COM_TIMER->SR = TIM_SR_UIF;
    loadCommutationTimer(2499);
    assert(!(COM_TIMER->CR1 & TIM_CR1_CEN));
    assert(!(COM_TIMER->DIER & TIM_DIER_UIE));
    assert(COM_TIMER->ARR == 2499 && COM_TIMER->EGR == TIM_EGR_UG);
    assert(COM_TIMER->SR == 0);
    startCommutationTimer(100);
    assert(COM_TIMER->CNT == 100 && (COM_TIMER->CR1 & TIM_CR1_CEN));
    assert(COM_TIMER->DIER & TIM_DIER_UIE);

    INTERVAL_TIMER->SR = TIM_SR_UIF;
    assert(commutationIntervalOverflowed());
    resetCommutationInterval(5000);
    assert(INTERVAL_TIMER->CNT == 5000);
    assert(!commutationIntervalOverflowed());
    resetCommutationInterval(9000);
    assert(INTERVAL_TIMER->CNT == 9000);
    puts("commutation boundary checks passed");
}
