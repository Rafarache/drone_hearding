import contextlib
import importlib.util
import io
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location('stop_lab', Path(__file__).parents[1] / 'stop_lab.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StopLabTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='lab stop ')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'ros_ws').mkdir()
        (self.root / 'scripts').mkdir()
        self.inspection = {'State': {'Running': True}, 'Mounts': [
            {'Type': 'bind', 'Source': str(self.root / 'ros_ws'), 'Destination': '/ros2_ws'}]}
        self.output = io.StringIO()
        redirect = contextlib.redirect_stdout(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)

    def controller(self, container):
        path = self.root / 'scripts/test_lab_service.py'
        path.write_text('import time\ntime.sleep(60)\n')
        process = subprocess.Popen([sys.executable, str(path), '--container', container])
        def cleanup():
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)
        self.addCleanup(cleanup)
        return process

    def test_gracefully_stops_matching_controller_before_container(self):
        server = self.controller('fixture')
        other = self.controller('other')
        def docker_command(args, **kwargs):
            if args[1] in ('exec', 'stop'):
                self.assertIsNotNone(server.poll(), 'Host controller must exit first')
                self.assertIsNone(other.poll(), 'Other container controllers must remain running')
            return subprocess.CompletedProcess(args, 0)
        with patch.object(module, 'inspect_container', return_value=self.inspection), \
             patch.object(module.subprocess, 'run', side_effect=docker_command) as docker:
            module.stop_lab(self.root, 'fixture', 3)
        self.assertEqual([call.args[0][1] for call in docker.call_args_list], ['info', 'exec', 'stop'])
        self.assertEqual(docker.call_args_list[-1].args[0], ['docker', 'stop', '--timeout', '3', 'fixture'])

    def test_missing_or_stopped_container_is_safe_to_stop_repeatedly(self):
        stopped = dict(self.inspection, State={'Running': False})
        for inspection in (None, stopped):
            with self.subTest(inspection=inspection), \
                 patch.object(module, 'inspect_container', return_value=inspection), \
                 patch.object(module.subprocess, 'run') as docker:
                module.stop_lab(self.root, 'fixture', 1)
                self.assertEqual([call.args[0][1] for call in docker.call_args_list], ['info'])

    def test_mismatched_checkout_is_rejected_before_signalling_processes(self):
        self.inspection['Mounts'][0]['Source'] = '/tmp/other-checkout'
        with patch.object(module, 'inspect_container', return_value=self.inspection), \
             patch.object(module.subprocess, 'run'), \
             patch.object(module, 'controllers') as controllers:
            with self.assertRaisesRegex(RuntimeError, 'does not mount this checkout'):
                module.stop_lab(self.root, 'fixture', 1)
            controllers.assert_not_called()

    def test_remaining_build_processes_are_cleaned_up_after_container_stops(self):
        controller = {'pid': 111, 'start': '1'}
        child = {'pid': 222, 'start': '2'}
        events = []
        def docker(args, **kwargs):
            events.append(args[1])
        def send(identities, sig):
            events.append(sig)
        with patch.object(module, 'inspect_container', return_value=self.inspection), \
             patch.object(module, 'controllers', return_value=[controller]), \
             patch.object(module, 'descendants', return_value=[child]), \
             patch.object(module, 'still_running', return_value=True), \
             patch.object(module, 'wait_for_exit', side_effect=[False, False, False, True]), \
             patch.object(module, 'send_signal', side_effect=send) as send_mock, \
             patch.object(module.subprocess, 'run', side_effect=docker):
            module.stop_lab(self.root, 'fixture', 1)
        self.assertLess(events.index('stop'), events.index(signal.SIGKILL))
        self.assertEqual(send_mock.call_args_list[-1].args, ([controller], signal.SIGKILL))

    def test_reused_pid_is_never_signalled(self):
        with patch.object(module, 'process_identity', return_value={'pid': 111, 'start': 'new'}), \
             patch.object(module.os, 'kill') as kill:
            module.send_signal([{'pid': 111, 'start': 'old'}], signal.SIGTERM)
        kill.assert_not_called()

    def test_docker_stop_failure_is_reported(self):
        def docker(args, **kwargs):
            if args[1] == 'stop':
                raise subprocess.CalledProcessError(1, args)
        with patch.object(module, 'inspect_container', return_value=self.inspection), \
             patch.object(module.subprocess, 'run', side_effect=docker):
            with self.assertRaises(subprocess.CalledProcessError):
                module.stop_lab(self.root, 'fixture', 1)
        self.assertNotIn('Server and container stopped', self.output.getvalue())


if __name__ == '__main__':
    unittest.main()
