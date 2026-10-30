"""Gracefully stop this checkout's lab controller, server, and Docker container."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def process_identity(pid):
    try:
        fields = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return {'pid': pid, 'ppid': int(fields[1]), 'start': fields[19]}
    except (OSError, ValueError, IndexError):
        return None


def still_running(identity):
    current = process_identity(identity['pid'])
    return current is not None and current['start'] == identity['start']


def controllers(root, container):
    expected = (root / 'scripts/test_lab_service.py').resolve()
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = entry.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0')
            if len(args) < 2 or not Path(args[0]).name.startswith('python'):
                continue
            script = Path(args[1])
            if not script.is_absolute():
                script = entry.joinpath('cwd').resolve() / script
            if script.resolve() != expected:
                continue
            selected = args[args.index('--container') + 1] if '--container' in args else 'drone_herding'
            identity = process_identity(int(entry.name))
            if selected == container and identity:
                result.append(identity)
        except (OSError, ValueError, IndexError, UnicodeError):
            continue
    return result


def descendants(parents):
    processes = [process_identity(int(entry.name)) for entry in Path('/proc').iterdir()
                 if entry.name.isdigit()]
    selected = {p['pid'] for p in parents if still_running(p)}
    children = []
    for _ in range(len(processes)):
        added = [p for p in processes if p and p['ppid'] in selected and p['pid'] not in selected]
        if not added:
            break
        children.extend(added)
        selected.update(p['pid'] for p in added)
    return children


def send_signal(identities, sig):
    for identity in identities:
        if still_running(identity):
            try:
                os.kill(identity['pid'], sig)
            except ProcessLookupError:
                pass


def wait_for_exit(identities, timeout):
    deadline = time.monotonic() + timeout
    while any(still_running(identity) for identity in identities):
        if time.monotonic() >= deadline:
            return False
        time.sleep(.1)
    return True


def inspect_container(container):
    result = subprocess.run(['docker', 'container', 'inspect', container], capture_output=True, text=True)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None


# Also handles a backend started directly, without the host controller.
STOP_BACKENDS = '''import os, pathlib, signal, sys, time
targets = []
for entry in pathlib.Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        command = entry.joinpath('cmdline').read_bytes().split(b'\\0')
        if b'simulation_test_pkg.server' not in command:
            continue
        start = entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]
        targets.append((entry, start))
        os.kill(int(entry.name), signal.SIGTERM)
    except (FileNotFoundError, ProcessLookupError):
        pass
deadline = time.monotonic() + float(sys.argv[1])
while targets and time.monotonic() < deadline:
    alive = []
    for entry, start in targets:
        try:
            fields = entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z' and fields[19] == start:
                alive.append((entry, start))
        except FileNotFoundError:
            pass
    targets = alive
    if targets:
        time.sleep(.1)
'''


def stop_lab(root, container, timeout):
    subprocess.run(['docker', 'info'], stdout=subprocess.DEVNULL, check=True)
    inspection = inspect_container(container)
    if inspection:
        mount = next((m for m in inspection.get('Mounts', []) if m['Destination'] == '/ros2_ws'), {})
        if (mount.get('Type') != 'bind' or
                Path(mount['Source']).resolve() != (root / 'ros_ws').resolve()):
            raise RuntimeError('The container does not mount this checkout at /ros2_ws. '
                               'Choose the correct SIM_TEST_CONTAINER or --container.')
    owned = controllers(root, container)
    children = descendants(owned) if owned else []
    if owned:
        print('[Herding Lab] Shutting down the server and active experiment...', flush=True)
        send_signal(owned, signal.SIGTERM)
        if not wait_for_exit(owned, timeout):
            print('[Herding Lab] Server shutdown is still pending; stopping its container next.', flush=True)
            children.extend(descendants([p for p in owned if still_running(p)]))
    inspection = inspect_container(container)
    if inspection and inspection['State']['Running']:
        print('[Herding Lab] Stopping remaining lab backends...', flush=True)
        subprocess.run(['docker', 'exec', container, 'python3', '-c', STOP_BACKENDS, str(timeout)], check=True)
        print(f'[Herding Lab] Stopping container {container}...', flush=True)
        subprocess.run(['docker', 'stop', '--timeout', str(math.ceil(timeout)), container], check=True)
    elif inspection:
        print(f'[Herding Lab] Container {container} is already stopped.', flush=True)
    else:
        print(f'[Herding Lab] Container {container} is already absent.', flush=True)
    # A controller may still be waiting for a build subprocess. Limit cleanup
    # to its recorded descendants, checking start times against PID reuse.
    if not wait_for_exit(owned, 2):
        send_signal(children, signal.SIGTERM)
        if not wait_for_exit([*children, *owned], 3):
            send_signal(children, signal.SIGKILL)
            send_signal(owned, signal.SIGKILL)
            if not wait_for_exit([*children, *owned], 2):
                raise RuntimeError('Some lab processes could not be stopped.')
    print('[Herding Lab] Server and container stopped. Saved runs are kept.', flush=True)


def main():
    parser = argparse.ArgumentParser(prog='stop_lab.sh', description=__doc__)
    parser.add_argument('--container', default=os.environ.get('SIM_TEST_CONTAINER', 'drone_herding'))
    parser.add_argument('--timeout', type=float, default=45,
                        help='seconds to allow each graceful shutdown step (default: 45)')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a positive finite number')
    try:
        stop_lab(Path(__file__).resolve().parents[1], args.container, args.timeout)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Could not stop Herding Lab: {error}\n')


if __name__ == '__main__':
    main()
