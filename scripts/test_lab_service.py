#!/usr/bin/env python3
"""Host-side build controller and restartable Docker lab server."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socketserver
import subprocess
import threading
import time
import urllib.request
import uuid


def now():
    return datetime.now(timezone.utc).isoformat()


def provenance(root):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()
    untracked = git('ls-files', '--others', '--exclude-standard').splitlines()
    return dict(branch=git('branch', '--show-current'), revision=git('rev-parse', 'HEAD'),
                dirty=bool(git('status', '--porcelain')), working_tree_patch=git('diff', 'HEAD'),
                untracked_files=untracked,
                untracked_sha256={name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                                  for name in untracked if (root / name).is_file()})


class LabService:
    def __init__(self, root, container, port):
        self.root = Path(root)
        self.workspace = self.root / 'ros_ws'
        self.data = self.workspace / 'test_results'
        self.data.mkdir(exist_ok=True)
        self.container, self.port = container, port
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.worker = None
        self.backend = None
        self.backend_identity = None
        self.state = dict(status='idle', message='Build the current checkout before testing changed code.')
        previous = self.data / 'build-state.json'
        if previous.exists():
            try:
                old = json.loads(previous.read_text())
                if old['status'] in {'building', 'restarting', 'failed'}:
                    self.state = dict(old, status='failed', message='Previous build did not finish successfully. Build again before testing.')
                elif old['status'] == 'succeeded':
                    self.state = old
            except (ValueError, KeyError):
                pass
        self.publish()

    def publish(self, **changes):
        with self.lock:
            self.state.update(changes)
            target = self.data / 'build-state.json'
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.state, indent=2))
            temporary.replace(target)

    def status(self):
        with self.lock:
            result = dict(self.state)
        try:
            with (self.data / 'build.log').open('rb') as log:
                log.seek(0, 2)
                log.seek(max(0, log.tell() - 30000))
                result['log'] = log.read().decode(errors='replace')
        except FileNotFoundError:
            result['log'] = ''
        return result

    def request(self, action):
        if action == 'status':
            return self.status()
        if action != 'build':
            return {'error': 'Unknown build action'}
        with self.lock:
            if self.stopping.is_set() or self.state['status'] in {'building', 'restarting'}:
                return {'error': 'A build is already running or the lab is stopping.'}
            # The API holds its submission lock while making this request.
            runs = self.api('runs')
            if any(run['status'] in {'queued', 'starting', 'running'} for run in runs):
                return {'error': 'Finish or cancel running and queued experiments before building.'}
            self.publish(status='building', id=uuid.uuid4().hex, started_at=now(), finished_at=None,
                         message='Building ROS packages and the website.', code=None)
            (self.data / 'build.log').write_text('')
            self.worker = threading.Thread(target=self.build, name='lab-build')
            self.worker.start()
            return self.status()

    def api(self, path):
        with urllib.request.urlopen(f'http://127.0.0.1:{self.port}/api/{path}', timeout=3) as response:
            return json.load(response)

    def command(self, args, log, cwd=None):
        log.write(('\n$ ' + ' '.join(str(a) for a in args) + '\n').encode())
        log.flush()
        subprocess.run(args, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, check=True)

    def start_backend(self):
        self.backend = subprocess.Popen([
            'docker', 'exec', '-i', self.container, 'bash', '-lc',
            'source /opt/ros/iron/setup.bash && source /ros2_ws/install/setup.bash && '
            'cd /ros2_ws && exec .test_lab_venv/bin/python -m simulation_test_pkg.server --port "$1"',
            '_', str(self.port)])
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.backend.poll() is not None:
                raise RuntimeError('Lab backend exited during startup.')
            try:
                health = self.api('health')
                if health.get('build_controller'):
                    self.backend_identity = health['process']
                    return
            except (OSError, ValueError):
                pass
            time.sleep(.2)
        raise RuntimeError('Lab backend did not become ready within 30 seconds.')

    def stop_backend(self):
        if not self.backend_identity:
            return
        # Validate PID start time and command before signalling inside Docker.
        code = '''import json, os, pathlib, signal, sys
identity=json.loads(sys.argv[1]);pid=identity['pid'];base=pathlib.Path('/proc')/str(pid)
try:
    start=base.joinpath('stat').read_text().rsplit(')',1)[1].split()[19]
    command=base.joinpath('cmdline').read_bytes().split(b'\\0')
    if start==identity['start'] and b'simulation_test_pkg.server' in command:
        os.kill(pid,signal.SIGTERM)
except FileNotFoundError:
    pass
'''
        subprocess.run(['docker', 'exec', self.container, 'python3', '-c', code,
                        json.dumps(self.backend_identity)], check=True)
        self.backend.wait(timeout=35)
        self.backend_identity = None

    def build(self):
        staging = self.workspace / 'src/simulation_test_pkg/web' / ('.lab-build-' + uuid.uuid4().hex)
        backup = staging.with_name(staging.name + '-previous')
        web = staging.parent
        try:
            before = provenance(self.root)
            self.publish(code={k: before[k] for k in ('branch', 'revision', 'dirty')})
            with (self.data / 'build.log').open('ab', buffering=0) as log:
                self.command(['docker', 'exec', self.container, 'bash', '-lc',
                              'source /opt/ros/iron/setup.bash && cd /ros2_ws && '
                              '.test_lab_venv/bin/python -m pip install -q -r src/simulation_test_pkg/requirements.lock && '
                              'colcon build'], log)
                self.command(['npm', 'ci', '--no-audit', '--no-fund'], log, web)
                self.command(['npm', 'run', 'build', '--', '--outDir', str(staging)], log, web)
                self.command(['docker', 'exec', self.container, 'bash', '-lc',
                              'source /opt/ros/iron/setup.bash && source /ros2_ws/install/setup.bash && '
                              'cd /ros2_ws && .test_lab_venv/bin/python -c '
                              '\'from simulation_test_pkg.server import create_app; create_app(manage_worker=False)\''], log)
                if provenance(self.root) != before:
                    raise RuntimeError('Checkout changed during the build. Build again before testing.')
                self.publish(status='restarting', message='Loading the new backend and website.')
                self.stop_backend()
                if (web / 'dist').exists():
                    (web / 'dist').rename(backup)
                staging.rename(web / 'dist')
                (self.data / 'checkout_provenance.json').write_text(json.dumps(before, indent=2))
                self.start_backend()
                self.publish(status='succeeded', message='Build complete. New experiments use this build.', finished_at=now())
        except Exception as error:
            if backup.exists() and not (web / 'dist').exists():
                backup.rename(web / 'dist')
            with (self.data / 'build.log').open('a') as log:
                log.write(f'\nBuild failed: {error}\n')
            self.publish(status='failed', message=str(error), finished_at=now())
            # Keep the UI available for reading logs and retrying after a failed restart.
            if self.backend is None or self.backend.poll() is not None:
                try:
                    self.start_backend()
                except Exception as restart_error:
                    self.publish(message=f'{error}; backend restart failed: {restart_error}')
        finally:
            for directory in (staging, backup):
                if directory.exists():
                    shutil.rmtree(directory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', default='drone_herding')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    data = root / 'ros_ws/test_results'
    data.mkdir(exist_ok=True)
    with (data / 'build-service.lock').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        service = LabService(root, args.container, args.port)
        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                try:
                    request = json.loads(self.rfile.readline(1000))
                    response = service.request(request.get('action'))
                except Exception as error:
                    response = {'error': str(error)}
                self.wfile.write((json.dumps(response) + '\n').encode())
        address = data / 'build-control.sock'
        address.unlink(missing_ok=True)
        with socketserver.ThreadingUnixStreamServer(str(address), Handler) as server:
            os.chmod(address, 0o600)
            server.daemon_threads = True
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            for signum in (signal.SIGINT, signal.SIGTERM):
                signal.signal(signum, lambda *_: service.stopping.set())
            try:
                service.start_backend()
                while not service.stopping.wait(.5):
                    if service.backend.poll() is not None and not (service.worker and service.worker.is_alive()):
                        raise RuntimeError('The lab backend stopped unexpectedly.')
            finally:
                service.stopping.set()
                if service.worker:
                    service.worker.join()
                service.stop_backend()
                server.shutdown()
                address.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
