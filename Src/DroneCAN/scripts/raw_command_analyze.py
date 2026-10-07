"""Offline endpoint comparison; uses the capture timestamps without retiming."""

import argparse
import csv
import json
from pathlib import Path


def analyze(directory):
    metadata = json.loads((directory / 'metadata.json').read_text())
    if metadata['stop_reason'] != 'complete' or metadata.get('reboot_detected'):
        raise ValueError('Incomplete run or ESC reboot: counter comparison invalid')
    with (directory / 'packet_timeline.csv').open() as stream:
        packets = list(csv.DictReader(stream))
    with (directory / 'command_submissions.csv').open() as stream:
        submissions = list(csv.DictReader(stream))
    source = str(metadata.get('test_options', {}).get('host_id', 127))
    results = []
    for case in metadata.get('counter_results', []):
        submitted = submissions[case['seq_start'] - 1:case['seq_end']]
        start = int(submitted[0]['real_before_ns'])
        end = int(submitted[-1]['real_after_ns']) + 50_000_000
        echo = [packet for packet in packets
                if packet['datatype'] == 'uavcan.equipment.esc.RawCommand'
                and packet['direction'] == 'local_tx_echo'
                and packet['source_node_id'] == source
                and packet['first_kernel_realtime_ns']
                and start <= int(packet['first_kernel_realtime_ns']) <= end]
        decoded = [packet for packet in echo if packet['status'] == 'decoded']
        wrong_slots = sum(len(json.loads(packet['payload_json'])['cmd']) != case['slots']
                          for packet in decoded)
        row = {key: case[key] for key in
               ('phase', 'slots', 'rate_hz', 'value', 'host_submissions', 'accepted')}
        missing = case['host_submissions'] - case['accepted']
        row.update(tx_echo_decoded=len(decoded), wrong_slot_count=wrong_slots,
                   missing=missing, missing_pct=100 * missing / case['host_submissions'],
                   rx_overflow_observations=(case['final']['rx_errors'] -
                                             case['baseline']['rx_errors']) & 0xffff,
                   comparable=len(decoded) == case['host_submissions'] and
                              wrong_slots == 0 and missing >= 0)
        results.append(row)
    (directory / 'comparison-derived.json').write_text(json.dumps(results, indent=2) + '\n')
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    options = parser.parse_args()
    print(json.dumps(analyze(options.directory), indent=2))
