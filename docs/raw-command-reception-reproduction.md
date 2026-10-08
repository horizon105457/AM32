# Low-speed RAW command reception: reproduction notes

## Images and fixture

Tested on 2026-10-08 with one VIMDRONES Nano L431 CAN (hardware 2.3), a
340 KV underwater ducted thruster, submerged and secured, supplied by 5S
(recorded voltage approximately 20.9 V). ESC node ID 1, sender node ID 127,
ESC_INDEX 0, classic CAN at 1 Mbit/s via a USB SLCAN bridge. Other array
entries were zero; no other physical ESCs were connected.

- Official base: `50234143a6531ef2ede1ac52c53f46f4dca1208a`.
- Baseline source: diagnostic-only commit
  `8fb890c8a67651d654f8472040af95bb4c7962dd`, based on that official commit.
  The baseline image was built before this commit was created, with the same
  diagnostic extension. It retains official commutation behavior.
- Candidate: `b657a9d0391d7b18d15b59fc23dddc5090506c73`, adding nonblocking
  polling commutation on top of the diagnostic commit.

Build each revision in a separate checkout/output directory:

```sh
make -B VIMDRONES_NANO_L431_CAN RAW_COMMAND_DEBUG=1 OBJ=obj/raw-debug
```

Candidate debug BIN SHA256:
`7ae11a4826e7e21103f4f6cb7c52342e2859187f1d48cb9da6620d25345081ce`.
Images were manually flashed by the operator. APP identified as
`com.vimdrones.esc_nano#M1`, software 2.21, with FlexDebug v3, but GetNodeInfo
commit/image CRC fields were absent: exact remote image identity was not verified.

## Recorded parameter baseline

These are the parameters read back for this comparison, not a complete EEPROM
dump. Retain other settings between images and select a safe command/load for
your own fixture; CMD163 is not a universal safe operating point.

| Key | Value | Key | Value |
| --- | --- | --- | --- |
| ESC_INDEX | 0 | MOTOR_POLES | 14 |
| MOTOR_KV | 340 | TELEM_RATE | 100 |
| INPUT_FILTER_HZ | 0 | VARIABLE_PWM | 0 |
| PWM_FREQUENCY | 16 | STALL_PROTECTION | 0 |
| MIN_DUTY_CYCLE | 4 | REQUIRE_ARMING | true |
| REQUIRE_ZERO_THROTTLE | true | BI_DIRECTIONAL | true |
| ADVANCE_LEVEL | 16 | AUTO_ADVANCE | false |
| DEBUG_RATE | 2 during capture, restored to 0 | | |

## Sender-independent procedure

Use any DroneCAN sender and raw-frame recorder capable of standard multi-frame
transfers; the local laboratory harness is not required. Keep source node ID,
transfer priority, load, settings and case order identical for both images.

1. Start recording raw TX/RX frames and host command submissions on one time
   basis. Record NodeStatus uptime, ESC.Status and FlexDebug. Send zero before
   arming to satisfy REQUIRE_ZERO_THROTTLE. Set DEBUG_RATE=2 and read it back.
2. Test CMD0 then positive CMD163. For each, use arrays of 8, 12 and 20 entries;
   for each length use 20, 50, 100 and 200 Hz. Send
   `uavcan.equipment.esc.RawCommand` with `cmd=[CMD,0,...,0]`, using valid DSDL
   serialization, transfer CRC and advancing transfer IDs. These lengths use
   3, 4 and 6 classic CAN frames respectively, including transfer CRC overhead.
3. Before each case, send no RAW commands for 0.8 s, while keeping
   `uavcan.equipment.safety.ArmingStatus` FULLY_ARMED alive (50 ms interval in
   the recorded test). Obtain a fresh cumulative-counter baseline.
4. Send RAW at the selected rate for eight seconds, with ArmingStatus at 2 Hz.
   Immediately send one additional zero RAW command with the same array
   length. Count that zero in this case. Expected counts are 161, 401, 801
   and 1601 at the four rates; count actual submissions and reconstructed
   complete TX transfers instead of assuming scheduler accuracy.
5. Allow 0.8 s without RAW for in-flight traffic/counter publication, keeping
   ArmingStatus alive as in step 3. Obtain the final counter snapshot. Reject
   a case if uptime resets, either endpoint is missing/stale, or TX transfer
   reconstruction is incomplete. Do not add uncounted setup RAW commands
   between the two counter endpoints.
6. Finish with zero/disarm, restore/read back DEBUG_RATE, and release the owned
   interface/serial resource. GetSet can trigger deferred EEPROM autosave;
   these parameter writes are not guaranteed purely volatile.

The baseline/candidate used this 24-case order. The candidate additionally had
a separate three-second running preflight, outside the matrix count; it could
prefill period history. Startup transients are therefore not strict A/B data.

## Counter decoding and meaning

Reassemble `dronecan.protocol.FlexDebug` (type ID 100), then decode its `u8`
array with [raw_command_debug.py](../Src/DroneCAN/scripts/raw_command_debug.py).
This file uses only Python's standard library. For example, after your sender
has decoded a complete FlexDebug message:

```python
# Put Src/DroneCAN/scripts on Python's module search path.
from raw_command_debug import decode, counter_delta
snapshot = decode(bytes(message.u8))
accepted = counter_delta(before['total_commands'], after['total_commands'])
fifo_observations = (after['rx_errors'] - before['rx_errors']) & 0xffff
```

Version 3 requires exactly 37 payload bytes. Little-endian uint32
`total_commands` starts at byte 29; `total_input` at byte 33. Reception count
means successful full RAW decoding containing this ESC's configured index,
including zero and arming-gated commands. It does **not** prove one execution
or physical actuation per command. `total_input` includes periodic 1 kHz
reinjection/failsafe and is unsuitable for this comparison. Cumulative deltas
are modulo 2^32 within one boot; missing intermediate debug packets do not
lose the cumulative count. More layout details are in
[RAW_command_count_test.md](../Src/DroneCAN/scripts/RAW_command_count_test.md).

On this STM32 driver, `rx_errors` increments when receive FIFO FOVR is observed.
Its delta counts overflow observations, not every lost CAN frame or RAW
transfer. `rxframe_error` instead concerns transfer processing. Compare:
`deficit = complete_TX_RAW_transfers - accepted`. Host enqueue success alone
does not prove transmission; local SocketCAN TX echo remains a host observation,
not physical bus EOF or a precise wire-latency measurement.

## Observed results

Each channel-length row aggregates the four running rates. Stationary rows were
complete before and after (2964/2964 per length, 8892/8892 overall).

| Running array length | Submitted / complete TX | Baseline accepted | Candidate accepted | Baseline deficit | Baseline / candidate FIFO observations |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8 | 2964 | 2957 | 2964 | 7 | 7 / 0 |
| 12 | 2964 | 2309 | 2964 | 655 | 654 / 0 |
| 20 | 2964 | 2252 | 2964 | 712 | 707 / 0 |
| All running | 8892 | 7518 | 8892 | 1374 | 1368 / 0 |

Across all 24 candidate cases: 17784 host submissions = complete local TX
transfers = accepted commands, with zero added FIFO-overflow observations.
Both captures reported zero socket overflow and zero missing kernel timestamps.
Eight channels are the application case; 12/20 channels are additional pressure
tests. This was one receiving ESC, not an eight-node cluster test.

Running baseline deficits by rate (all candidate deficits were zero):

| Array length | 20 Hz: 161 sent | 50 Hz: 401 sent | 100 Hz: 801 sent | 200 Hz: 1601 sent |
| --- | ---: | ---: | ---: | ---: |
| 8 | 2 | 0 | 3 | 2 |
| 12 | 34 | 73 | 189 | 359 |
| 20 | 37 | 106 | 193 | 376 |

Tail period medians changed from 19157–20037.5 to 17771.5–18813 timer ticks;
reported tail RPM changed from 142 to 157 in several cases. Demand fields
remained identical (adjusted_input 82, duty_cycle 73). Display quantization
alone does not explain the period change; physical speed versus detection or
commutation timing effects remain unresolved without independent measurement.
No physical tachometer, ISR WCET, masked-duration or ZC-to-commutation timing
measurement was made. Reversal, all-speed operation and other board hardware
were not validated. Standard telemetry still had incomplete/invalid reconstructed
transfers; this is not an end-to-end telemetry loss-free claim.

## Evidence provenance and harness boundary

Original full captures remain local, excluded from Git. The compact results
above derive from their cumulative endpoint records, not summed delta packets.

| Capture | Raw frames SHA256 | Metadata SHA256 |
| --- | --- | --- |
| Baseline: `obj/raw-command-baseline-2026-10-08/run-073717/` | `6605d854a4437b32a087f42a3650bc5bd3446fde132df43a569cd172b98c122c` | `b1e3e44d5be8b3337d64cb39831d023e9cd97a7609cd605afe4cdf712dbb22de` |
| Candidate: `obj/repaired-raw-matrix-20261008-145428/run/` | `46aa910da6b6e2df8fe2f48766790043e9bff7bc8fef75314e07074fd5f86868` | `8c2320d73db31c94731d1e615ce69082a4cc664ef6aa8c1d3c0d2abee49e1e03` |

The committed `raw_command_capture.py` depends on the separate
`hexbot-esc-test` project and is not a standalone public reproduction tool.
The repaired-image capture used a local compatibility copy: SLCAN ownership
matching was corrected to avoid counting sudo wrappers, and the decoder import
path was adjusted. The committed capture script was **not** used unmodified.
The counter method above avoids that harness dependency. The separate initial
baseline run `run-073619` rebooted and is excluded from these results.
