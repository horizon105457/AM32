# RAW command reception accounting

Build with `RAW_COMMAND_DEBUG=1` to enable the optional extension. The default
`RAW_COMMAND_DEBUG=0` preserves the official v2 payload and debug scheduling.
Use a separate output directory and `make -B` when changing this option:

```sh
make -B VIMDRONES_NANO_L431_CAN RAW_COMMAND_DEBUG=1 OBJ=obj/raw-debug
```

With the extension compiled, FlexDebug ID 100 carries AM32 payload version 3. The version 1/2 prefix
and delta counters are unchanged. Two little-endian uint32 fields are appended:

| Byte offset (zero based) | Field | Meaning |
| --- | --- | --- |
| 29 | total_commands | RAW transfers decoded successfully and containing this ESC's configured index |
| 33 | total_input | All set_input calls, including periodic 1 kHz refresh and failsafe |

The payload is 37 bytes. Counts start at zero on boot and wrap modulo 2^32.
Zero throttle and commands gated by arming are counted: these counters prove
reception, not physical motor actuation. `total_input` is not one-to-one
with RAW commands: it also includes periodic 1 kHz reinjection and failsafe. No new counter or
receive interrupt work is added. Snapshotting uses the existing CAN IRQ exclusion
in DroneCAN_update; no additional interrupt masking is introduced.

Set DEBUG_RATE to 2 Hz initially (5 Hz if needed), keeping TELEM_RATE and the
standard RAW channel count unchanged. DEBUG_RATE=0 disables FlexDebug. Each
debug transfer uses six classic CAN frames, so include this traffic in the bus
capture, particularly when testing a cluster. The updated ArduPilot Lua example
accepts versions 1/2/3 and logs cumulative values as AMD3; its node range remains
the original four-node example and must be configured for the actual cluster.
The debug scheduler skips missed periods instead of emitting catch-up bursts
after delayed service or enabling DEBUG_RATE late in a session.

Record host send requests, actual bus TX/RX frames, and reconstructed RAW and
FlexDebug transfers on one clock. Use cumulative differences modulo 2^32 rather
than summing the older delta fields: a lost debug transfer then loses temporal
resolution, not the intervening command count. Split records on ESC reboot
(NodeStatus uptime reset); do not interpret a reset as ordinary wraparound.

For exact totals, stop test commands, allow in-flight transfers to drain, record
a counter baseline, send a known number of valid RAW transfers from one source,
stop sending, and wait for a final counter snapshot. Preserve normal arming and
failsafe behavior. Compare the baseline/final difference with bus-observed full
RAW transfers that include the ESC index. Host enqueue success alone is not
proof of bus transmission. A missing final debug packet is inconclusive until
another snapshot arrives. Periodic snapshots alone cannot locate each lost
command precisely.

Compare stationary and approved running conditions with the same channel count,
command rate and telemetry settings. Eight-channel RAW is the primary case;
twenty-channel traffic is a separate pressure test. Reception count deficits
establish a command-processing gap, not its cause. Retain RX error/FIFO evidence
to distinguish hardware overflow from transfer reassembly or other losses.

This diagnostic branch changes no commutation, RAW mapping, failsafe, standard
ESC telemetry, EEPROM layout, or generated DSDL. It precedes the independent
nonblocking-commutation change so the comparison can remain single-factor.


## Reusable test tools

`raw_command_capture.py` requires the local hexbot-esc-test repository and its
Python dependencies/DSDL definitions, plus Linux SocketCAN, slcand and permission
to operate the selected interface. `--help` opens no device. Without `--run` it
refuses to acquire CAN. It refuses existing interfaces or occupied serial ports,
creates its own SLCAN connection, records all traffic and host requests, then
sends zero/disarm, restores DEBUG_RATE and releases its connection on exit.
Only run after the fixture and permitted CMD/duration have been confirmed.
Default CMD=0 tests stationary reception only. Set `--cmd` explicitly for motion.
This is a laboratory tool; host termination or power loss cannot guarantee cleanup.

```sh
python3 Src/DroneCAN/scripts/raw_command_capture.py --help
python3 Src/DroneCAN/scripts/raw_command_capture.py --run \
  --tool /path/to/hexbot-esc-test --output obj/my-raw-baseline \
  --channels 8 12 20 --rates 20 50 100 200 --duration 8 --cmd 163
python3 Src/DroneCAN/scripts/raw_command_analyze.py obj/my-raw-baseline
python3 Src/DroneCAN/scripts/test_raw_command_debug.py
```

Use `--expected-params` with a JSON object to require a known parameter baseline;
the tool records relevant parameters but does not otherwise change them except
DEBUG_RATE (temporarily 2 Hz). GetSet writes mark settings dirty and official
firmware may autosave once stopped, even without an explicit SAVE request. Restore
is checked and recorded. Keep TELEM_RATE identical for before/after comparison.
The capture requires complete v3 debug snapshots before running and standard
ESC.Status feedback while running. It uses ArmingStatus keepalive at counter
boundaries without adding RAW commands; uptime regression invalidates the run.
Boundary waits are 0.8 s at debug 2 Hz; reboot detection remains sampled, so also
check raw NodeStatus and startup records. Modulo subtraction assumes the same
boot and fewer than 2^32 events per interval. Results with incomplete local TX
reconstruction are marked non-comparable. TX echoes are host observations and
cannot establish physical delivery or precise wire latency.

See `RAW_command_baseline.md` for the existing hardware baseline and hashes.
This commit contains no new hardware test. Lua logging requires ArduPilot and
was not runtime-tested here.


## Commit validation

Nano CAN OFF and ON ARM builds passed with GCC 10.3.1. OFF uses 52,173 B Flash
and 8,224 B RAM; ON uses 52,213 B Flash and 8,232 B RAM (+40 B / +8 B).
The OFF BIN is byte-identical to the preserved upstream 5023414 Nano baseline:
SHA256 `8acb1c423261dafeb0799bec158871b99f193c7e7f770fe39ed266360731b932`.
The Makefile emits existing F421 duplicate-recipe warnings, with no compiler
warnings. Host checks compile the actual packed C struct with both option values,
decode its bytes, reject truncated payloads, exercise wrap and dropped debug
samples, and verify the CLI refuses acquisition without --run. Existing 24-case
capture reanalysis matches all TX endpoint counts. Build outputs/logs remain in
`obj/raw-command-commits/`; Lua was not runtime-tested.
