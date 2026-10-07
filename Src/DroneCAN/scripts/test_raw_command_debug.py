"""Host-only wire-layout/accounting checks; never opens CAN or a serial port."""

import pathlib
import subprocess
import tempfile
import unittest

from raw_command_debug import COUNTERS, PREFIX, counter_delta, decode


class DebugTests(unittest.TestCase):
    def test_actual_firmware_layout(self):
        source = pathlib.Path(__file__).parents[1].joinpath('DroneCAN.c').read_text()
        definition = source[source.index('static struct PACKED {'):]
        definition = definition[:definition.index('} debug1;') + len('} debug1;')]
        with tempfile.TemporaryDirectory() as temp:
            c = pathlib.Path(temp, 'layout.c')
            exe = pathlib.Path(temp, 'layout')
            c.write_text('#include <stdint.h>\n#include <stdio.h>\n'
                         '#define PACKED __attribute__((__packed__))\n' + definition +
                         '\nint main(void) { debug1.version = RAW_COMMAND_DEBUG ? 3 : 2;\n'
                         '#if RAW_COMMAND_DEBUG\n'
                         'debug1.total_commands = 0xfffffffe; debug1.total_input = 42;\n'
                         '#endif\n'
                         'return fwrite(&debug1, sizeof(debug1), 1, stdout) != 1; }\n')
            for flag in (0, 1):
                subprocess.run(['cc', '-Wall', '-Werror',
                                f'-DRAW_COMMAND_DEBUG={flag}', str(c), '-o', str(exe)],
                               check=True)
                payload = subprocess.check_output([str(exe)])
                self.assertEqual(len(payload), 37 if flag else 29)
                decoded = decode(payload)
                self.assertEqual(decoded['version'], 3 if flag else 2)
                if flag:
                    self.assertEqual(decoded['total_commands'], 0xfffffffe)
                    self.assertEqual(decoded['total_input'], 42)

    def test_loss_of_intermediate_debug_sample(self):
        # Dropping the intermediate sample changes time resolution, not totals.
        self.assertEqual(counter_delta(123, 300), 177)
        self.assertEqual(counter_delta(123, 180) + counter_delta(180, 300), 177)

    def test_wrap(self):
        self.assertEqual(counter_delta(0xfffffffe, 3), 5)

    def test_truncated_or_unknown(self):
        prefix = PREFIX.pack(3, *([0] * 13))
        with self.assertRaises(ValueError):
            decode(prefix)
        with self.assertRaises(ValueError):
            decode(bytes([4]) + prefix[1:] + COUNTERS.pack(1, 1))
        self.assertEqual(decode(prefix + COUNTERS.pack(3, 9))['total_input'], 9)

    def test_cli_requires_run_before_importing_hardware(self):
        script = pathlib.Path(__file__).with_name('raw_command_capture.py')
        with tempfile.TemporaryDirectory() as temp:
            target = pathlib.Path(temp, 'must-not-be-created')
            result = subprocess.run(['python3', str(script), '--tool', '/missing',
                                     '--output', str(target)], capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn(b'No hardware action without --run', result.stderr)
            self.assertFalse(target.exists())


if __name__ == '__main__':
    unittest.main()
