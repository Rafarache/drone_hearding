import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('lab_service', Path(__file__).parents[1] / 'test_lab_service.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BuildServiceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'ros_ws').mkdir()
        self.service = module.LabService(self.root, 'fixture', 8765)
        self.service.backend = Mock()
        self.service.backend.poll.return_value = None
        self.code = dict(branch='feature', revision='abc123', dirty=True,
                         working_tree_patch='patch', untracked_files=[])

    def build_command(self, args, log, cwd=None):
        if args[:3] == ['npm', 'run', 'build']:
            target = Path(args[-1])
            target.mkdir(parents=True)
            (target / 'index.html').write_text('new website')

    def test_builds_ros_and_web_then_restarts_and_publishes_provenance(self):
        with patch.object(module, 'provenance', return_value=self.code), \
             patch.object(self.service, 'command', side_effect=self.build_command) as command, \
             patch.object(self.service, 'stop_backend') as stop, \
             patch.object(self.service, 'start_backend') as start:
            self.service.build()
        self.assertEqual(self.service.state['status'], 'succeeded')
        self.assertEqual(command.call_count, 4)
        self.assertIn('colcon build', command.call_args_list[0].args[0][-1])
        stop.assert_called_once();start.assert_called_once()
        self.assertEqual((self.root / 'ros_ws/src/simulation_test_pkg/web/dist/index.html').read_text(), 'new website')
        self.assertTrue((self.service.data / 'checkout_provenance.json').exists())

    def test_compiler_failure_preserves_running_backend_and_records_failure(self):
        with patch.object(module, 'provenance', return_value=self.code), \
             patch.object(self.service, 'command', side_effect=RuntimeError('compiler failed')), \
             patch.object(self.service, 'stop_backend') as stop:
            self.service.build()
        self.assertEqual(self.service.state['status'], 'failed')
        self.assertIn('compiler failed', self.service.status()['log'])
        stop.assert_not_called()

    def test_checkout_change_during_build_is_rejected(self):
        with patch.object(module, 'provenance', side_effect=[self.code, dict(self.code, revision='changed')]), \
             patch.object(self.service, 'command', side_effect=self.build_command), \
             patch.object(self.service, 'stop_backend') as stop:
            self.service.build()
        self.assertEqual(self.service.state['status'], 'failed')
        self.assertIn('Checkout changed', self.service.state['message'])
        stop.assert_not_called()

    def test_failed_state_survives_controller_restart(self):
        self.service.publish(status='building')
        replacement = module.LabService(self.root, 'fixture', 8765)
        self.assertEqual(replacement.state['status'], 'failed')


if __name__ == '__main__':
    unittest.main()
