"""Manage only processes positively identified as belonging to a run."""
import os
from pathlib import Path
import signal
import subprocess
import time


def process_identity(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return {'pid': int(pid), 'start': text[19], 'ppid': int(text[1])}
    except (OSError, ValueError, IndexError):
        return None


def still_owned(identity):
    current = process_identity(identity['pid']) if identity else None
    return current is not None and current['start'] == identity['start']


def descendants(pid):
    processes = [process_identity(int(p.name)) for p in Path('/proc').iterdir() if p.name.isdigit()]
    children, parents = [], {pid}
    while True:
        found = [p for p in processes if p and p['ppid'] in parents and p['pid'] not in parents]
        if not found:
            return children
        children += found
        parents.update(p['pid'] for p in found)


def terminate_launch(process):
    children = descendants(process.pid)
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        if process.poll() is None:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
        break
    # Some launch actions use separate process groups. Never use global pkill.
    for identity in reversed(children):
        if still_owned(identity):
            try:
                os.kill(identity['pid'], signal.SIGTERM)
            except ProcessLookupError:
                pass
    time.sleep(.1)
    for identity in reversed(children):
        if still_owned(identity):
            try:
                os.kill(identity['pid'], signal.SIGKILL)
            except ProcessLookupError:
                pass


def cleanup_lease(path):
    import json
    try:
        lease = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return
    launch = lease.get('launch')
    children = lease.get('descendants', []) + lease.get('auxiliary', [])
    if still_owned(launch):
        children += descendants(launch['pid'])
        try:
            os.killpg(launch['pid'], signal.SIGTERM)
        except ProcessLookupError:
            pass
    for identity in reversed(children):
        if still_owned(identity):
            try:
                os.kill(identity['pid'], signal.SIGTERM)
            except ProcessLookupError:
                pass
    display = lease.get('display')
    if still_owned(display):
        try:
            os.kill(display['pid'], signal.SIGTERM)
        except ProcessLookupError:
            pass
    worker = lease.get('worker')
    if still_owned(worker):
        try:
            os.kill(worker['pid'], signal.SIGTERM)
        except ProcessLookupError:
            pass
