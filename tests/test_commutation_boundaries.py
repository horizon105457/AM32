"""Compile the real header with register stubs and enforce recovery source rules."""

from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as temporary:
    folder = Path(temporary)
    header = ROOT / 'Inc/commutation.h'
    (folder / 'commutation.h').write_bytes(header.read_bytes())
    (folder / 'targets.h').write_text('')
    (folder / 'peripherals.h').write_text('')
    executable = folder / 'boundaries'
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-O2',
                    '-I', str(folder), str(ROOT / 'tests/commutation_boundaries.c'),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)

    # Compile the actual main-loop recovery block with observable side effects.
    # Do not recreate its branch conditions or estimator formula in the test.
    main = (ROOT / 'Src/main.c').read_text()
    start = main.index('            uint32_t recovery_period = INTERVAL_TIMER_COUNT;')
    end = main.index('#else', start)
    recovery = main[start:end]
    prefix = (ROOT / 'tests/commutation_boundaries.c').read_text().split('int main(void)')[0]
    fixture = prefix + '''
enum { COM_NONE, COM_POLL };
static unsigned commutation_pending, commutation_epoch;
static unsigned running = 1, input = 163, old_routine, zcfound, zero_crosses;
static unsigned zero_throttle_brake_active, bemf_timeout_happened, temp_comp_pwm;
static uint32_t commutation_interval = 12500;
static uint16_t thiszctime = 123;
static unsigned immediate_calls, estimator_calls;
static struct { unsigned comp_pwm; } eepromBuffer;
static void maskPhaseInterrupts(void) {}
static void cancelCommutationLocked(void) {
    commutation_pending = COM_NONE; commutation_epoch++; zcfound = 0;
}
static void PeriodElapsedCallback(void) {
    assert(commutation_pending == COM_POLL && zero_crosses == 0);
    commutation_pending = COM_NONE; immediate_calls++; zcfound = 0;
}
static void schedulePollingCommutation(uint32_t epoch) {
    assert(epoch == commutation_epoch); estimator_calls++;
}
#define __set_PRIMASK(v) ((void)(v))
static void recovery(void) {
    uint32_t primask = 0;
''' + recovery + '''
}
int main(void) {
    INTERVAL_TIMER->CNT = 3; INTERVAL_TIMER->SR = TIM_SR_UIF;
    commutation_pending = COM_POLL;
    recovery();
    assert(immediate_calls == 1 && estimator_calls == 0);
    assert(thiszctime == 123 && commutation_interval == 12500);
    assert(commutation_pending == COM_NONE); /* callback final phase state not modeled */
    assert(INTERVAL_TIMER->CNT == 0 && !commutationIntervalOverflowed());
    recovery(); /* consumed overflow must not trigger a second recovery */
    assert(immediate_calls == 1 && estimator_calls == 0);
    INTERVAL_TIMER->CNT = 45000; recovery();
    assert(estimator_calls == 0);
    INTERVAL_TIMER->CNT = 45001; recovery();
    assert(estimator_calls == 1 && thiszctime == 45001);
    assert(immediate_calls == 1 && bemf_timeout_happened == 2);
    puts("production recovery branch checks passed (register model)");
}
'''
    recovery_source = folder / 'recovery.c'
    recovery_source.write_text(fixture)
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-O2',
                    '-I', str(folder), str(recovery_source), '-o', str(executable)],
                   check=True)
    subprocess.run([str(executable)], check=True)
