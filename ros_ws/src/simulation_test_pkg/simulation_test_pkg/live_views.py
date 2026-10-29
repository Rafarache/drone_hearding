"""Owned desktop viewers and non-blocking ROS window parameter controls."""
import json
import os
from pathlib import Path
import subprocess
import time
from .models import Visualizations
from .processes import process_identity
from .storage import atomic_json, utcnow


class LiveViews:
    def __init__(self, directory, manifest, node, environment, recording, virtual_display=False):
        self.directory = Path(directory)
        self.node, self.environment, self.recording = node, environment, recording
        self.virtual_display = virtual_display
        self.settings = Visualizations.model_validate(manifest['scenario'].get('visualization', {})).model_dump()
        self.revision = 'initial'
        self.processes, self.clients, self.futures, self.applied, self.failed = {}, {}, {}, {}, {}
        self.errors = {}
        self.stop_deadlines = {}
        self.logs = {}
        self.last_write = 0
        self.targets = {e['name'] + f"/grid{e['index']}/set_parameters": {
            'show_occupancy_grid': 'occupancy', 'show_vector_grid': 'vectors', 'show_coverage_map': 'visited'}
            for e in manifest['entities'] if e['kind'] == 'drone'}
        self.targets['/yolo_pkg/set_parameters'] = {
            'show_annotated_camera': 'camera', 'show_realtime_track_plot': 'tracker'}

    def tick(self):
        try:
            request = json.loads((self.directory / 'live_request.json').read_text())
            if request['revision'] != self.revision:
                self.settings = Visualizations.model_validate(request['settings']).model_dump()
                self.revision = request['revision']
                self.errors.clear()
                self.recording.writer.event(self.recording.elapsed, 'live_visualizations', json.dumps(self.settings))
        except FileNotFoundError:
            pass
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        for target, parameters in self.targets.items():
            future = self.futures.get(target)
            if future:
                revision, call = future
                if not call.done():
                    continue
                try:
                    results = call.result().results
                    if len(results) != len(parameters) or not all(r.successful for r in results):
                        raise RuntimeError('; '.join(r.reason for r in results if not r.successful) or 'Parameters rejected')
                    self.applied[target] = revision
                    self.errors.pop(target, None)
                except Exception as error:
                    self.errors[target] = str(error)
                    self.failed[target] = revision
                self.futures.pop(target, None)
            if self.applied.get(target) == self.revision or self.failed.get(target) == self.revision:
                continue
            client = self.clients.setdefault(target, self.node.create_client(SetParameters, target)) if target not in self.clients else self.clients[target]
            if client.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [Parameter(name, Parameter.Type.BOOL, self.settings[key]).to_parameter_msg()
                                      for name, key in parameters.items()]
                self.futures[target] = (self.revision, client.call_async(request))
        self._viewers()
        if time.monotonic() - self.last_write >= .25:
            stable = all(self.applied.get(t) == self.revision for t in self.targets)
            stable = stable and all(bool(self.processes.get(key) and self.processes[key].poll() is None) == self.settings[key]
                                    for key in ('gazebo', 'rviz'))
            atomic_json(self.directory / 'live_state.json', {'settings': self.settings, 'revision': self.revision,
                        'available': True, 'applied': stable and not self.errors, 'errors': list(self.errors.values()),
                        'display': self.environment.get('DISPLAY'), 'virtual_display': self.virtual_display,
                        'updated_at': utcnow()})
            self.last_write = time.monotonic()

    def _viewers(self):
        from ament_index_python.packages import get_package_share_directory
        commands = {'gazebo': ['gzclient'], 'rviz': ['rviz2', '-d',
                    str(Path(get_package_share_directory('sjtu_drone_bringup')) / 'rviz/rviz.rviz'),
                    '--ros-args', '-p', 'use_sim_time:=true', '-r', '__node:=test_lab_rviz']}
        for key, command in commands.items():
            process = self.processes.get(key)
            if process and process.poll() is not None:
                if self.settings[key] and key not in self.stop_deadlines:
                    self.failed[key] = self.revision
                    self.errors[key] = f'{key} viewer exited ({process.returncode}); see {key}.log'
                self.processes.pop(key, None)
                self.stop_deadlines.pop(key, None)
                process = None
            if self.settings[key]:
                if not process and self.failed.get(key) != self.revision:
                    if key in self.logs:
                        self.logs[key].close()
                    log = (self.directory / f'{key}.log').open('a')
                    self.logs[key] = log
                    try:
                        self.processes[key] = subprocess.Popen(command, env=self.environment, stdout=log,
                                                              stderr=subprocess.STDOUT)
                    except OSError as error:
                        self.errors[key] = str(error)
                        self.failed[key] = self.revision
            elif process:
                if key not in self.stop_deadlines:
                    process.terminate()
                    self.stop_deadlines[key] = time.monotonic() + 3
                elif time.monotonic() >= self.stop_deadlines[key]:
                    process.kill()

    def identities(self):
        return [identity for p in self.processes.values() if p.poll() is None
                if (identity := process_identity(p.pid))]

    def close(self):
        for process in self.processes.values():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        for log in self.logs.values():
            log.close()
        for client in self.clients.values():
            self.node.destroy_client(client)
