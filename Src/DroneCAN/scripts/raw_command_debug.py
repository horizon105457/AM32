"""Decode AM32 FlexDebug payloads and compare cumulative uint32 counters."""

import struct

PREFIX = struct.Struct('<BIHHHHiBHHHHHB')
COUNTERS = struct.Struct('<II')
FIELDS = ('version', 'commutation_interval', 'num_commands', 'num_input',
          'rx_errors', 'rxframe_error', 'rx_ecode', 'auto_advance_level',
          'duty_cycle', 'duty_cycle_maximum', 'adjusted_input',
          'adc_raw_current', 'adc_raw_volts', 'flags')


def decode(data):
    """Require complete v2/v3 payloads; do not silently accept truncated v3."""
    if not data or data[0] not in (2, 3):
        raise ValueError('AM32 payload version 2 or 3 required')
    expected = PREFIX.size + (COUNTERS.size if data[0] == 3 else 0)
    if len(data) != expected:
        raise ValueError('Incorrect AM32 debug payload length')
    values = dict(zip(FIELDS, PREFIX.unpack_from(data)))
    if data[0] == 3:
        values.update(zip(('total_commands', 'total_input'),
                          COUNTERS.unpack_from(data, PREFIX.size)))
    return values


def counter_delta(before, after):
    """Modulo subtraction requires the same boot and less than 2^32 events."""
    return (after - before) & 0xffffffff
