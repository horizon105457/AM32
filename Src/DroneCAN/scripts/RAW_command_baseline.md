# 2026-10-08 official-base RAW reception baseline

Image: official upstream 5023414 plus pre-commit FlexDebug v3 diagnostic extension. User flashed the image. No nonblocking commutation fix.

One ESC; CMD=0/163, 8/12/20 channels, 20/50/100/200 Hz, eight seconds per case plus a final zero command. Standard telemetry 100 Hz, debug 2 Hz. All stopped cases: 0% deficit. Running results:

| Channels | 20 Hz | 50 Hz | 100 Hz | 200 Hz |
| --- | ---: | ---: | ---: | ---: |
| 8 | 1.24% | 0% | 0.37% | 0.12% |
| 12 | 21.12% | 18.20% | 23.60% | 22.42% |
| 20 | 22.98% | 26.43% | 24.09% | 23.49% |

All 24 endpoint comparisons match host submissions to reconstructed local TX echoes. No cumulative counter resets. ESC driver FIFO-overflow observations increased in running cases: 20 channels/100 Hz had 193 missing commands and 193 overflow observations. This establishes hardware FIFO overflow, but does not identify the blocking ISR or count every lost frame. One ESC receiving standard multichannel messages is not eight-node cluster acceptance.

153,317 raw frames, zero capture socket overflow, zero missing kernel timestamps. SLCAN local echo proves host-origin observation, not physical bus EOF. 58 backwards timestamps in receive order are retained; no precise latency claim is made. Standard telemetry reconstruction also had incomplete/invalid transfers; debug cumulative endpoints tolerate lost intermediate debug messages.

The first run run-073619 is invalid: boundary silence restarted APP and reset counters. The valid run used ArmingStatus keepalive during RAW silence. num_input includes periodic 1 kHz set_input refresh and is not a command count.

Cleanup recorded zero/disarm, DEBUG_RATE restored to 0, interface gone and serial unowned. Parameter GetSet writes can trigger official deferred EEPROM autosave; parameter changes are not purely volatile.

Original artifacts remain local and untracked at `obj/raw-command-baseline-2026-10-08/run-073717/`. File hashes bind this summary to that capture; large CSVs and generated plots are intentionally excluded from Git.

| Artifact | SHA256 |
| --- | --- |
| raw_frames.csv | `6605d854a4437b32a087f42a3650bc5bd3446fde132df43a569cd172b98c122c` |
| metadata.json | `b1e3e44d5be8b3337d64cb39831d023e9cd97a7609cd605afe4cdf712dbb22de` |
| command_submissions.csv | `2d4466cdaeea93f8dd9d8dd6eee5d99e5ce2265f1bf2f8f75fe3fd21f0716d4e` |
| packet_timeline.csv | `1e36e28409caf2a84ad0decc6d41373038c1b36c4384c8d55b0a81cb611eb012` |
| comparison.json | `3787261b350f92f47c0b685d6518d57d5caa8282158f90137073ba22c063ae46` |
