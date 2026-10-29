"""A durable sequential queue, independent of HTTP requests and browser tabs."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from .configuration import snapshot
from .models import BatchSpec, Scenario
from .processes import cleanup_lease, process_identity, still_owned
from .storage import Store, TERMINAL, identifier, utcnow, database


def submit_batch(store, spec, snapshotter=snapshot):
    spec = spec if isinstance(spec, BatchSpec) else BatchSpec.model_validate(spec)
    prepared, created = [], []
    try:
        for item in spec.items:
            saved = store.scenario(item.scenario_id) if item.scenario_id else None
            if item.scenario_id and saved is None:
                raise ValueError(f'Scenario not found: {item.scenario_id}')
            scenario = item.scenario or Scenario.model_validate({k: v for k, v in saved.items()
                                                                 if k in Scenario.model_fields})
            for repetition in range(item.repeats):
                run_id = identifier()
                directory = store.run_dir(run_id)
                created.append(directory)
                manifest = snapshotter(directory, scenario)
                prepared.append((run_id, item.scenario_id,
                                 {'scenario': scenario.model_dump(), 'entities': manifest['entities'],
                                  'repetition': repetition + 1, 'scenario_id': item.scenario_id}))
        return store.enqueue(spec.name, prepared)
    except BaseException:
        import shutil
        for directory in created:
            if directory.exists():
                shutil.rmtree(directory)
        raise


class Supervisor:
    def __init__(self, store, worker_command=None):
        self.store = store
        self.worker_command = worker_command or [sys.executable, '-m', 'simulation_test_pkg.worker']
        self.stopping = threading.Event()
        self.thread = None
        self.process = None
        self.active = None
        self.lock = None

    def start(self):
        self.lock = (self.store.root / 'supervisor.lock').open('a+')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            self.lock = None
            raise RuntimeError('Another test runner already owns this results directory')
        with database(self.store.path) as db:
            unfinished = [row['id'] for row in db.execute(
                "SELECT id FROM runs WHERE status IN ('starting','running')")]
        for run_id in unfinished:
            directory = self.store.run_dir(run_id)
            cleanup_lease(directory / 'process.json')
            try:
                state = json.loads((directory / 'state.json').read_text())
            except (OSError, ValueError):
                state = {}
            if state.get('status') not in TERMINAL:
                state.update(status='interrupted', finished_at=utcnow(),
                             reason='Previous supervisor stopped before finalizing this run')
            self.store.update(run_id, state)
        self.thread = threading.Thread(target=self._loop, name='experiment-supervisor', daemon=True)
        self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=35)
        if self.process and self.process.poll() is None:
            cleanup_lease(self.store.run_dir(self.active) / 'process.json')
            self.process.kill()
            self.process.wait(timeout=5)
        if self.lock:
            fcntl.flock(self.lock, fcntl.LOCK_UN)
            self.lock.close()
            self.lock = None

    def _cancel_worker(self):
        try:
            lease = json.loads((self.store.run_dir(self.active) / 'process.json').read_text())
            identity = lease.get('worker')
        except (OSError, ValueError):
            identity = process_identity(self.process.pid)
        if still_owned(identity):
            try:
                os.kill(identity['pid'], signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _loop(self):
        while not self.stopping.is_set():
            run = self.store.claim()
            if run is None:
                self.stopping.wait(.25)
                continue
            self.active = run['id']
            directory = self.store.run_dir(self.active)
            environment = dict(os.environ, ROS_DOMAIN_ID=str(201 + int(self.active[:4], 16) % 20))
            try:
                with (directory / 'worker.log').open('a') as log:
                    self.process = subprocess.Popen([*self.worker_command, '--run-dir', str(directory)],
                                                    env=environment, stdout=log, stderr=subprocess.STDOUT,
                                                    start_new_session=True)
                    cancel_started = None
                    while self.process.poll() is None:
                        latest = self.store.run(self.active)
                        if self.stopping.is_set() or latest.get('cancel_requested'):
                            if cancel_started is None:
                                self._cancel_worker()
                                cancel_started = time.monotonic()
                            if time.monotonic() - cancel_started > 25:
                                cleanup_lease(directory / 'process.json')
                                self.process.kill()
                                break
                        time.sleep(.2)
                    self.process.wait(timeout=5)
                try:
                    state = json.loads((directory / 'state.json').read_text())
                except (OSError, ValueError):
                    state = {}
                if state.get('status') not in TERMINAL:
                    cleanup_lease(directory / 'process.json')
                    cancelled = self.stopping.is_set() or self.store.run(self.active).get('cancel_requested')
                    state.update(status='cancelled' if cancelled else 'failed', finished_at=utcnow(),
                                 reason='Worker stopped before finalization; see worker log')
                self.store.update(self.active, state)
            except BaseException as error:
                cleanup_lease(directory / 'process.json')
                self.store.update(self.active, {'status': 'failed', 'reason': str(error), 'finished_at': utcnow()})
            finally:
                self.process = None
                self.active = None
