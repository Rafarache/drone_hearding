"""Local build-controller bridge and experiment/build exclusion."""
from contextlib import contextmanager
import json
import socket
import threading


class BuildConflict(RuntimeError):
    pass


class BuildControl:
    def __init__(self, store):
        self.store = store
        self.lock = threading.RLock()

    def _request(self, action):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(5)
            connection.connect(str(self.store.root / 'build-control.sock'))
            connection.sendall((json.dumps({'action': action}) + '\n').encode())
            with connection.makefile('r') as stream:
                response = json.loads(stream.readline(200000))
        if 'error' in response:
            raise BuildConflict(response['error'])
        return response

    def status(self):
        try:
            return dict(self._request('status'), available=True)
        except (OSError, ValueError):
            try:
                previous = json.loads((self.store.root / 'build-state.json').read_text())
            except (OSError, ValueError):
                previous = {'status': 'idle'}
            return dict(previous, available=False,
                        message='Restart the lab with ./scripts/test_lab.sh to enable builds.')

    @contextmanager
    def submission(self):
        with self.lock:
            if self.status()['status'] in {'building', 'restarting', 'failed'}:
                raise BuildConflict('Complete a successful build before starting new experiments.')
            yield

    def start(self):
        with self.lock:
            if any(run['status'] in {'queued', 'starting', 'running'} for run in self.store.runs()):
                raise BuildConflict('Finish or cancel running and queued experiments before building.')
            try:
                return self._request('build')
            except (OSError, ValueError) as error:
                raise BuildConflict('Build controller unavailable. Restart with ./scripts/test_lab.sh.') from error
