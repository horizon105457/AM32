# Nonblocking polling commutation

The official-base command-loss evidence is in
`Src/DroneCAN/scripts/RAW_command_baseline.md`: running 12/20-channel RAW
commands showed acceptance deficits and hardware receive-FIFO overflow; the
stationary cases did not. This repair removes the polling commutation wait from
the periodic interrupt, using the existing COM timer. It changes neither
DroneCAN framing/priority/FIFO depth nor EEPROM/PWM/input-filter configuration.

## Timing and cancellation

- One event owner (NONE/POLL/COMP), no software queue. Polling captures and
  resets INTERVAL once. Further polls cannot replace the pending event.
- The original polling IIR and advance calculation run outside IRQ masking.
  The first five crossings remain immediate. Later crossings use
  ARR=waitTime-1, CNT=elapsed since capture. Already-expired delays or interval
  overflow complete immediately. Immediate completion retains event ownership.
- Loading stops COM, forces active ARR/PSC loading, clears UIF, reads fresh
  elapsed time and starts COM in one short PRIMASK-preserving transaction.
  ADC cannot interrupt between the final elapsed read and COM start.
- Normal captures must have CNT in 1..45000 and no UIF before CNT/UIF reset.
  Overflowed low bits never replace thiszctime/lastzctime or enter the IIR.
  Invalid capture leaves CNT/UIF available to existing main-loop recovery;
  comparator is masked and no pending event is left frozen. A zero-period
  capture also waits for that recovery instead of inventing a period.
- The original 45000-tick recovery threshold remains. Main also recognizes UIF
  so a delayed service cannot miss the threshold after wrap. Unwrapped timeout
  recovery keeps the original sampled-period formula; wrapped recovery keeps
  the prior estimate, executes the original immediate first recovery step, and
  starts a fresh CNT/UIF epoch. No synthetic period, wrap counter or split timer.
- Delay arithmetic is checked at uint32 width before conversion: interval is
  1..65535 and configured advance 0..32, so delay is at most 32767; COMP +1 fits
  uint16_t. POLL -1 is used only for nonzero delay.
- StartMotor retains its 5000 seed and sine handover its 9000 seed, now clearing
  stale UIF together with CNT. Old-poll startup now starts CNT at zero in the
  same short transaction as running=1; its first sample reference therefore
  changes from arbitrary stopped-time CNT to elapsed time since startup. First
  five crossings remain immediate and steady-state estimator formulas unchanged.
- COMP retains its own estimator/advance formulas and ARR=waitTime+1 convention.
  It does not execute the polling IIR.
- COM handlers require UIF and UIE and clear UIF before dispatch. They do not
  clear shared NVIC pending state. Consumption and phase/comparator-register
  commitment are atomic. A cancellation epoch rejects stale completion writes
  after high-priority stop/reversal, without queuing or retries.
- Stop, reversal, sine transitions, desync, voltage/signal/BEMF timeout and
  DroneCAN direction-parameter changes cancel old events. RC reversal retains
  its original mode; normal reversal returns to polling. Zero-throttle coasting
  continues tracking. The original immediate BEMF-timeout recovery is retained.

The supported GPIO/comparator helpers contain no wait/delay loops. Register and
state transactions preserve PRIMASK; period calculations are outside them.
Actual masked time and ISR worst-case duration still require board measurement.

## Scope and evidence

STM32 F031/F051/G031/G071/L431/G431 and AT32F421 use the new route. E230 keeps
legacy polling: enabling the initial repair exceeded its application flash
budget by 2,244 bytes. AT32F415 retains legacy polling because the two timer
prescalers differ; NXP, WCH and SITL also retain legacy paths. Legacy commutation
and polling bodies remain separate to preserve small-target code generation.
No vendor HAL is modified.

Final checks use GCC 10.3.1 and separate output directories with make -B:
GD32DEV_A_E230, AM32REF_F051, AT32DEV_F421, GEN_128K_G071,
VIMDRONES_NANO_L431_CAN, SEQURE_G431_CAN, REF_F031, GEN_G031 and REF_F415_CAN.
Nano also has a RAW_COMMAND_DEBUG=1 build. Logs and source hashes are under
`obj/nonblocking-commutation-final-20261008-135357/`. The five host diagnostic tests check
wire layout/accounting, not motor timing. tests/test_commutation_boundaries.py
compiles the actual timer/math helpers, exhaustively checks representable delays,
and compiles the production main-loop recovery branch with register stubs.
It checks overflow keeps the estimate and routes one immediate recovery, while
unwrapped timeout routes the sampled period to the estimator. The callback is
a stub: phase changes, final zero_crosses and ISR interleavings are not simulated.
These models do not measure ISR or physical timing. Independent review covered preload,
cancellation, flags and preemption; coordinator closeout preserved immediate
ownership and narrowed E230 scope. The repaired image is not hardware-tested.

After flashing the debug-enabled repaired Nano, repeat the original
8/12/20-channel, 20/50/100/200 Hz stationary/CMD163 matrix with identical motor
parameters, TELEM_RATE=100 and DEBUG_RATE=2. Verify counter continuity, TX echo
counts, FIFO-overflow deltas and acceptance deficits. Retain raw frames on one
clock; local TX echo does not establish physical bus timing. Also measure
ZC-to-commutation delay/masked time and check startup, reversal, coast and
sine/mode handover. Single-node results are not eight-node cluster acceptance.

## Final build record

All ten configurations above passed; source/build-input hashes were unchanged
through the builds. No compiler warnings; existing Makefile F421 duplicate
recipe warnings remain. Build records identify pre-commit HEAD `8fb890c8`; they
validate these exact source hashes, not an already-created repair commit.

| Configuration | Flash bytes |
| --- | ---: |
| nano | 54269 |
| nano-debug | 54309 |
| e230 | 27260 |
| f051 | 24740 |
| f421 | 24660 |
| f031 | 25208 |
| g071 | 26936 |
| g031 | 26524 |
| g431 | 57229 |
| f415 | 51609 |

Nano OFF: Flash 54269 B, RAM 8232 B, BIN 54813 B, SHA256
`0d1872d6953b81542793ffb9b3ad413eee49dd89b8488970c2e1fc3c8b95ff71`.
Nano debug ON: Flash 54309 B, RAM 8240 B, BIN 54853 B, SHA256
`7ae11a4826e7e21103f4f6cb7c52342e2859187f1d48cb9da6620d25345081ce`.
The debug-enabled image is under `nano-debug/` in the final output directory;
use it for the identical command-count comparison after flashing. No hardware
operation or repair-image hardware validation was performed in this closeout.
