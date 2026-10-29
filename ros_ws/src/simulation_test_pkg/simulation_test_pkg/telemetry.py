"""Deterministic recorder state machine; ROS transport is kept in worker.py."""
import math
import time


class Recording:
    def __init__(self, manifest, writer, monotonic=time.monotonic):
        self.manifest, self.writer, self.monotonic = manifest, writer, monotonic
        self.scenario = manifest['scenario']
        self.expected = {e['name']: e for e in manifest['entities']}
        self.clock = self.origin = None
        self.last_sample = -1.0
        self.last_bucket = -1
        self.latest = {}
        self.outcome = self.reason = None
        self.created_wall = self.last_clock_wall = self.last_full_wall = monotonic()
        self.ready_wall = None
        self.received_stop = False
        self.pending_search = {}

    @property
    def elapsed(self):
        if self.origin is None or self.clock is None:
            return 0.0
        return min(self.scenario['duration_s'], max(0.0, self.clock - self.origin))

    def update_clock(self, clock):
        if not math.isfinite(clock) or self.outcome:
            return
        if self.clock is not None and clock < self.clock - 1e-8:
            self.finish('failed', 'Simulation clock reset unexpectedly')
            return
        if self.clock is None or clock > self.clock:
            self.last_clock_wall = self.monotonic()
        self.clock = clock
        if self.origin is not None and clock - self.origin >= self.scenario['duration_s']:
            # Do not manufacture a final pose at the deadline from an older frame.
            self.finish('timed_out', 'Simulation-time limit reached')

    def update_positions(self, entities):
        if self.outcome or self.clock is None:
            return
        self.latest = {e['name']: e for e in entities
                       if e['name'] in self.expected and
                       all(math.isfinite(v) for v in [*e['pose'], *e['velocity']])}
        complete = len(self.latest) == len(self.expected)
        if complete:
            self.last_full_wall = self.monotonic()
        if self.origin is None:
            if not complete:
                return
            self.origin = self.clock
            self.ready_wall = self.monotonic()
            self.writer.event(0, 'ready', 'All expected models and the simulation clock are available')
            for name, state in self.pending_search.items():
                self._record_search(name, state)
            self.pending_search.clear()
        t = self.elapsed
        if math.floor((t + 1e-8) * 10) > self.last_bucket:
            self.sample(t)
        goal = self.scenario['goal']
        cows = [self.latest.get(name) for name, e in self.expected.items() if e['kind'] == 'cow']
        if cows and all(e and math.hypot(e['pose'][0] - goal['x'], e['pose'][1] - goal['y']) <= goal['radius_m']
                        for e in cows):
            self.sample(t)
            self.finish('success', 'All configured cows reached the goal in one snapshot')

    def sample(self, t):
        self.writer.frame(round(t, 9), list(self.latest.values()))
        self.last_sample = t
        self.last_bucket = math.floor((t + 1e-8) * 10)

    def update_detections(self, points):
        if self.origin is None or self.outcome:
            return
        finite = [[x, y] for x, y in points if math.isfinite(x) and math.isfinite(y)]
        self.writer.detections(self.elapsed, finite)

    def update_search(self, drone, value):
        if self.outcome or self.expected.get(drone, {}).get('kind') != 'drone':
            return
        from .search import normalize_state
        try:
            state = normalize_state(value)
            if self.origin is None:
                previous = self.pending_search.get(drone)
                if previous is None or state['stamp_s'] >= previous['stamp_s']:
                    self.pending_search[drone] = state
            else:
                self._record_search(drone, state)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.writer.event(self.elapsed, 'search_rejected', str(error))

    def _record_search(self, drone, state):
        t = max(0.0, state['stamp_s'] - self.origin)
        if t <= self.scenario['duration_s']:
            self.writer.search.record(drone, round(t, 9), state)

    def check_watchdogs(self, startup_s=120, silence_s=60):
        now = self.monotonic()
        if self.origin is None:
            if now - self.created_wall >= startup_s:
                self.finish('failed', 'Startup timed out waiting for the clock and all expected models')
        elif now - self.last_clock_wall >= silence_s:
            self.finish('failed', 'Simulation clock stopped advancing')
        elif now - self.last_full_wall >= silence_s:
            self.finish('failed', 'Expected model telemetry is missing')

    def finish(self, status, reason):
        if self.outcome:
            return
        self.outcome, self.reason = status, reason
        self.writer.event(self.elapsed, status, reason)
        self.writer.flush()

    def state(self):
        now = self.monotonic()
        return {'status': self.outcome or ('running' if self.origin is not None else 'starting'),
                'reason': self.reason, 'simulation_duration_s': self.elapsed,
                'real_duration_s': now - self.created_wall,
                'startup_duration_s': (self.ready_wall or now) - self.created_wall,
                'recorded_models': len(self.latest), 'expected_models': len(self.expected),
                'received_mission_stop': self.received_stop}
