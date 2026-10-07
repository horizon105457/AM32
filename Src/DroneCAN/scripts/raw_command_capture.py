#!/usr/bin/env python3
"""Exclusive RAW reception test. CLI parsing precedes hardware imports."""
from pathlib import Path
import sys, time, subprocess, threading, socket, csv, json, bisect, argparse, os, math, struct
ap = argparse.ArgumentParser(description='Exclusive SLCAN RAW reception test; --run required')
ap.add_argument('--run', action='store_true')
ap.add_argument('--tool', type=Path, required=True, help='hexbot-esc-test repository')
ap.add_argument('--output', type=Path, required=True, help='new artifact directory')
ap.add_argument('--serial', default='/dev/ttyACM0')
ap.add_argument('--interface', default='slcan7')
ap.add_argument('--node-id', type=int, default=1)
ap.add_argument('--host-id', type=int, default=127)
ap.add_argument('--esc-index', type=int, default=0)
ap.add_argument('--channels', type=int, nargs='+', default=[8, 12, 20])
ap.add_argument('--rates', type=int, nargs='+', default=[20, 50, 100, 200])
ap.add_argument('--duration', type=float, default=8)
ap.add_argument('--cmd', type=int, default=0, help='approved positive running CMD; zero tests stopped only')
ap.add_argument('--rpm-max', type=int, default=1000)
ap.add_argument('--expected-params', type=Path, help='optional JSON object of required parameter values')
args = ap.parse_args()
if not args.run:
    ap.error('No hardware action without --run; read the test instructions first')
if not (0 <= args.esc_index < min(args.channels) <= max(args.channels) <= 20 and 1 <= min(args.rates) <= max(args.rates) <= 200 and (args.duration >= 3) and (0 <= args.cmd <= 8191) and (1 <= args.node_id <= 126) and (1 <= args.host_id <= 127) and (args.node_id != args.host_id) and (args.rpm_max > 0)):
    ap.error('Invalid channel/rate/node/motion limits')
TOOL = args.tool.resolve()
OUT = args.output.resolve()
IF = args.interface
sys.path.insert(0, str(TOOL / 'src'))
import dronecan
from esc_test.am32.telemetry import ensure_dsdl_loaded
from esc_test.capture import open_capture_socket, frame_row, FIELDS, clock_sample
from esc_test.capture_decode import packet_timeline, FIELDS as PFIELDS
from esc_test.config import load_yaml
from esc_test.node import build_node_info
from esc_test.can_ops import SpinGuard, boot_app_via_commands
from esc_test.params_cli import maintenance_spin
from esc_test.params_io import read_param, write_param, ParamChange
from dronecan.app.node_monitor import NodeMonitor
from raw_command_debug import decode, counter_delta
ensure_dsdl_loaded()
OUT.mkdir(parents=True, exist_ok=False)
rows = []
submits = []
phases = []
stopped = threading.Event()
sock = None
reader = None
node = None
monitor = None
owned = []
laststatus = [None]
lastpayload = [None]
lastvalid = [None]
restore_required = False
debug_original = None
active_slots = 20
debug_rows = []
lastdebug = [None]
meta = {'start_clock': clock_sample(), 'run_requested': args.run, 'timestamp_limit': 'SocketCAN kernel CLOCK_REALTIME software timestamps. TX echo is host-origin observation, not physical CAN EOF. SLCAN/USB bridge contributes delay.', 'current_firmware': 'FlexDebug v3 required; image SHA supplied externally.', 'test_options': {k: str(v) if isinstance(v, Path) else v for (k, v) in vars(args).items()}, 'stop_reason': 'complete'}

def pid_rows():
    text = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    return [(int(line.split(None, 1)[0]), line.split(None, 1)[1]) for line in text.splitlines()[1:] if 'slcand ' in line and args.serial + ' ' + IF in line]

def arm(value):
    m = dronecan.uavcan.equipment.safety.ArmingStatus()
    m.status = m.STATUS_FULLY_ARMED if value else m.STATUS_DISARMED
    node.broadcast(m)

def command(value, phase='zero', rate=10):
    m = dronecan.uavcan.equipment.esc.RawCommand()
    m.cmd = [0] * active_slots
    m.cmd[args.esc_index] = value
    before = time.time_ns()
    mono = time.monotonic_ns()
    node.broadcast(m)
    after = time.time_ns()
    submits.append({'seq': len(submits) + 1, 'phase': phase, 'rate_hz': rate, 'value': value, 'slot_count': active_slots, 'real_before_ns': before, 'real_after_ns': after, 'mono_before_ns': mono})

def recv():
    try:
        while not stopped.is_set():
            try:
                (data, anc, flags, addr) = sock.recvmsg(72, socket.CMSG_SPACE(16) + socket.CMSG_SPACE(4))
            except socket.timeout:
                continue
            rows.append(frame_row(data, anc, flags, addr, IF, len(rows) + 1, time.monotonic_ns(), time.time_ns()))
    except BaseException as exc:
        meta['capture_error'] = repr(exc)
        stopped.set()

def hold(value, duration, rate, label, armed, check=False, shutdown=False):
    start = time.monotonic()
    nextcmd = start
    nextarm = start
    phase = {'name': label, 'rate_hz': rate, 'cmd': value, 'slot_count': active_slots, 'start_real_ns': time.time_ns(), 'start_mono_ns': time.monotonic_ns()}
    phases.append(phase)
    while time.monotonic() - start < duration:
        now = time.monotonic()
        if now >= nextarm:
            arm(armed)
            nextarm = now + 0.5
        if now >= nextcmd:
            command(value, label, rate)
            nextcmd += 1 / rate
            if nextcmd < now:
                nextcmd = now + 1 / rate
        spinner.spin(max(0, min(0.002, nextcmd - time.monotonic())))
        if stopped.is_set() and (not shutdown):
            raise RuntimeError('Capture reader stopped')
        if check and now - start > 2:
            if lastdebug[0] is None or now - lastdebug[0] > 2.5:
                raise RuntimeError('No usable FlexDebug for 2.5 seconds; stopping')
            if laststatus[0] is None or now - laststatus[0] > 1.5:
                raise RuntimeError('No fresh standard ESC.Status for 1.5 seconds; stopping')
            if lastvalid[0] is None or now - lastvalid[0] > 2:
                raise RuntimeError('No valid speed feedback for 2 seconds; stopping without changing throttle')
            if lastpayload[0] and (not 1 <= lastpayload[0]['rpm'] <= args.rpm_max):
                raise RuntimeError('RPM outside configured running guard; stopping')
    phase.update(end_real_ns=time.time_ns(), end_mono_ns=time.monotonic_ns())
try:
    if subprocess.run(['ip', 'link', 'show', IF], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError(IF + ' already exists')
    if subprocess.run(['fuser', args.serial], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError(args.serial + ' already in use')
    if pid_rows():
        raise RuntimeError('Existing matching slcand owner')
    log = (OUT / 'slcand.log').open('w')
    proc = subprocess.Popen(['sudo', '-n', 'slcand', '-o', '-c', '-f', '-s8', '-S', '460800', '-t', 'hw', '-F', args.serial, IF], stdout=log, stderr=subprocess.STDOUT)
    for _ in range(50):
        if subprocess.run(['ip', 'link', 'show', IF], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            break
        time.sleep(0.1)
    owned = [pid for (pid, _) in pid_rows()]
    if len(owned) != 1:
        raise RuntimeError('Expected one exclusively created slcand process')
    meta['owned_slcand_pids'] = owned
    subprocess.run(['sudo', '-n', 'ip', 'link', 'set', IF, 'txqueuelen', '1000'], check=True)
    subprocess.run(['sudo', '-n', 'ip', 'link', 'set', IF, 'up'], check=True)
    (sock, fd) = open_capture_socket(IF, 4 * 1024 * 1024)
    sock.settimeout(0.1)
    reader = threading.Thread(target=recv)
    reader.start()
    config = load_yaml(TOOL / 'config/bringup/slcan-host.yaml')
    node = dronecan.make_node(IF, node_id=args.host_id, node_info=build_node_info(config['host']))
    monitor = NodeMonitor(node)
    spinner = SpinGuard(node)

    def on_status(event):
        if int(event.transfer.source_node_id) == args.node_id:
            laststatus[0] = time.monotonic()
            lastpayload[0] = {'rpm': int(event.message.rpm), 'current': float(event.message.current), 'esc_index': int(event.message.esc_index)}
            lastvalid[0] = laststatus[0]
    node.add_handler(dronecan.TYPENAMES['uavcan.equipment.esc.Status'], on_status)

    def on_debug(event):
        if int(event.transfer.source_node_id) != args.node_id or int(event.message.id) != 100:
            return
        data = bytes(event.message.u8)
        if len(data) != 37 or data[0] != 3:
            meta['unexpected_debug_payload'] = {'length': len(data), 'hex': data.hex()}
            return
        values = decode(data)
        lastdebug[0] = time.monotonic()
        debug_rows.append(dict(values, dispatch_real_ns=time.time_ns(), dispatch_mono_ns=time.monotonic_ns(), transfer_id=int(event.transfer.transfer_id), kernel_first_frame_real=event.transfer.ts_real))
    node.add_handler(dronecan.TYPENAMES['dronecan.protocol.FlexDebug'], on_debug)
    last_uptime = [None]

    def on_node_status(event):
        if int(event.transfer.source_node_id) != args.node_id:
            return
        uptime = int(event.message.uptime_sec)
        if last_uptime[0] is not None and uptime < last_uptime[0]:
            meta['reboot_detected'] = True
        last_uptime[0] = uptime
    node.add_handler(dronecan.TYPENAMES['uavcan.protocol.NodeStatus'], on_node_status)
    app = boot_app_via_commands(node, monitor, args.node_id, spinner, timeout_sec=10, arming=False, command_slots=20)
    meta['app'] = app
    if not app['ok']:
        raise RuntimeError('APP unavailable under zero/disarmed command stream')
    parameters = []
    for key in ('ESC_INDEX', 'MOTOR_POLES', 'MOTOR_KV', 'TELEM_RATE', 'INPUT_FILTER_HZ', 'VARIABLE_PWM', 'PWM_FREQUENCY', 'STALL_PROTECTION', 'MIN_DUTY_CYCLE', 'REQUIRE_ARMING', 'REQUIRE_ZERO_THROTTLE', 'BI_DIRECTIONAL', 'ADVANCE_LEVEL', 'AUTO_ADVANCE', 'DEBUG_RATE'):
        row = None
        for attempt in range(3):
            row = read_param(node, args.node_id, key, spin=maintenance_spin(node, spinner))
            if row is not None:
                break
        if row is None:
            raise RuntimeError('Parameter read failed: ' + key)
        parameters.append(row)
    meta['parameters'] = parameters
    p = {row['name']: row['value'] for row in parameters}
    print(json.dumps({'output': str(OUT), 'app': app, 'params': p}, default=str), flush=True)
    if args.run:
        expected = json.loads(args.expected_params.read_text()) if args.expected_params else {}
        expected['ESC_INDEX'] = args.esc_index
        differences = {key: {'actual': p[key], 'expected': v} for (key, v) in expected.items() if p[key] != v}
        if differences:
            raise RuntimeError('Unexpected preflight parameter values: ' + str(differences))
        debug_original = int(p['DEBUG_RATE'])
        restore_required = True

        def service_spin(value, armed, label):
            last = [0.0]
            lastarm = [0.0]

            def spin(timeout):
                now = time.monotonic()
                if now - last[0] >= 0.01:
                    command(value, label, 100)
                    last[0] = now
                if now - lastarm[0] >= 0.5:
                    arm(armed)
                    lastarm[0] = now
                spinner.spin(min(timeout, 0.005))
            return spin
        setup_spin = service_spin(0, False, 'debug_setup')
        wr = write_param(node, args.node_id, ParamChange('DEBUG_RATE', 2), spin=setup_spin)
        rb = read_param(node, args.node_id, 'DEBUG_RATE', spin=setup_spin)
        meta['debug_rate_setup'] = {'original': debug_original, 'write': wr, 'readback': rb}
        if rb is None or int(rb['value']) != 2:
            raise RuntimeError('DEBUG_RATE1 readback failed')
        hold(0, 3, 20, 'debug_online_verification', False)
        if lastdebug[0] is None or time.monotonic() - lastdebug[0] > 1.5:
            raise RuntimeError('No recent usable FlexDebug while stopped; refusing motor test')
        hold(0, 1.5, 20, 'arming_zero', True)
        meta['counter_results'] = []
        for value in (0, args.cmd) if args.cmd else (0,):
            for slots in args.channels:
                for rate in args.rates:
                    active_slots = slots
                    label = f"{('stopped' if value == 0 else 'running')}_slots{slots}_{rate}hz"
                    until = time.monotonic() + 0.8
                    next_keepalive = 0
                    while time.monotonic() < until:
                        now = time.monotonic()
                        if now >= next_keepalive:
                            arm(True)
                            next_keepalive = now + 0.05
                        spinner.spin(0.005)
                    if not debug_rows or time.monotonic() - lastdebug[0] > 1:
                        raise RuntimeError('Missing counter baseline')
                    baseline = dict(debug_rows[-1])
                    seq0 = len(submits)
                    print('Starting ' + label, flush=True)
                    hold(value, args.duration, rate, label, True, check=value != 0)
                    command(0, label + '_stop', rate)
                    until = time.monotonic() + 0.8
                    next_keepalive = 0
                    while time.monotonic() < until:
                        now = time.monotonic()
                        if now >= next_keepalive:
                            arm(True)
                            next_keepalive = now + 0.05
                        spinner.spin(0.005)
                    final = dict(debug_rows[-1])
                    n = len(submits) - seq0
                    if meta.get('reboot_detected'):
                        raise RuntimeError('ESC uptime reset; counter endpoints invalid')
                    result = {'phase': label, 'slots': slots, 'rate_hz': rate, 'value': value, 'host_submissions': n, 'accepted': counter_delta(baseline['total_commands'], final['total_commands']), 'input_completed': counter_delta(baseline['total_input'], final['total_input']), 'baseline': baseline, 'final': final, 'seq_start': seq0 + 1, 'seq_end': len(submits)}
                    meta['counter_results'].append(result)
                    (OUT / 'progress.json').write_text(json.dumps(meta['counter_results'], indent=2))
                    print(json.dumps({k: v for (k, v) in result.items() if k not in ('baseline', 'final')}), flush=True)
        meta['last_ack_payload'] = lastpayload[0]
    else:
        hold(0, 3, 20, 'zero_disarmed', False)
    meta['transfer_errors'] = spinner.transfer_errors
    meta['interface_counters'] = subprocess.check_output(['ip', '-s', '-details', 'link', 'show', IF], text=True)
except BaseException as exc:
    meta['stop_reason'] = 'error'
    meta['error'] = repr(exc)
    print('ERROR ' + repr(exc), flush=True)
finally:
    if node is not None:
        for (label, duration, armed) in [('stop_zero', 1, args.run), ('stop_disarmed', 0.5, False)]:
            try:
                hold(0, duration, 20, label, armed, shutdown=True)
            except BaseException as exc:
                meta.setdefault('shutdown_command_errors', []).append(repr(exc))
        if restore_required:
            try:
                last = [0.0]

                def restore_spin(timeout):
                    now = time.monotonic()
                    if now - last[0] >= 0.05:
                        command(0, 'restore_debug_rate', 20)
                        arm(False)
                        last[0] = now
                    spinner.spin(min(timeout, 0.01))
                wr = None
                rb = None
                for attempt in range(3):
                    wr = write_param(node, args.node_id, ParamChange('DEBUG_RATE', debug_original), spin=restore_spin)
                    rb = read_param(node, args.node_id, 'DEBUG_RATE', spin=restore_spin)
                    if rb is not None and int(rb['value']) == debug_original:
                        break
                meta['debug_rate_final_restore'] = {'write': wr, 'readback': rb, 'verified': rb is not None and int(rb['value']) == debug_original, 'save_sent': False, 'original_debug_rate': debug_original, 'firmware_deferred_autosave': 'GetSet writes mark dirty; safe stopped firmware may autosave after 500ms. No claim of purely volatile changes.'}
            except BaseException as exc:
                meta['debug_rate_restore_error'] = repr(exc)
        try:
            meta['telem_rate_final_readback'] = read_param(node, args.node_id, 'TELEM_RATE', spin=restore_spin if restore_required else maintenance_spin(node, spinner))
        except BaseException as exc:
            meta['telem_rate_readback_error'] = repr(exc)
        try:
            if monitor is not None:
                monitor.close()
            node.close()
        except BaseException as exc:
            meta['node_close_error'] = repr(exc)
    if reader is not None:
        time.sleep(0.2)
        stopped.set()
        reader.join(timeout=2)
    if sock is not None:
        sock.close()
    if owned:
        subprocess.run(['sudo', '-n', 'ip', 'link', 'set', IF, 'down'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for pid in owned:
            subprocess.run(['sudo', '-n', 'kill', '-TERM', str(pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(30):
            if subprocess.run(['ip', 'link', 'show', IF], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
                break
            time.sleep(0.1)
    meta['cleanup'] = {'interface_absent': subprocess.run(['ip', 'link', 'show', IF], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0, 'owned_slcand_remaining': pid_rows(), 'tty_unowned': subprocess.run(['fuser', args.serial], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0}
    meta['end_clock'] = clock_sample()
    meta['frames'] = len(rows)
    meta['phases'] = phases
    meta['missing_kernel_timestamps'] = sum((r['kernel_realtime_ns'] == '' for r in rows))
    meta['rxq_overflow_reported_total'] = max([int(r['rxq_overflow_total']) for r in rows if r['rxq_overflow_total'] != ''] or [0])
    meta['error_frames'] = sum((r['err'] == 1 for r in rows))
    with (OUT / 'raw_frames.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    with (OUT / 'command_submissions.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['seq', 'phase', 'rate_hz', 'value', 'slot_count', 'real_before_ns', 'real_after_ns', 'mono_before_ns'])
        w.writeheader()
        w.writerows(submits)
    if debug_rows:
        with (OUT / 'flexdebug_online.csv').open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(debug_rows[0]))
            w.writeheader()
            w.writerows(debug_rows)
    meta['raw_capture_note'] = 'Dedicated receive thread stores all frames in memory. CSV serialized after stop; no disk IO on command/capture path.'
    (OUT / 'metadata.json').write_text(json.dumps(meta, indent=2, default=str) + '\n')
    (packets, decode) = packet_timeline([{k: str(v) for (k, v) in r.items()} for r in rows])
    with (OUT / 'packet_timeline.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=PFIELDS)
        w.writeheader()
        w.writerows(packets)
    (OUT / 'decode_summary.json').write_text(json.dumps(decode, indent=2) + '\n')
    print(json.dumps({'output': str(OUT), 'stop_reason': meta['stop_reason'], 'cleanup': meta['cleanup'], 'frames': len(rows)}), flush=True)
if meta['stop_reason'] != 'complete':
    raise SystemExit(1)
