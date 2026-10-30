import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
DOCKER = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ['LAUNCH_TEST_LOG'], 'a') as log:
    log.write(json.dumps(['docker', *args]) + '\\n')
settings = json.loads(Path(os.environ['LAUNCH_TEST_SETTINGS']).read_text())
if args[:2] == ['container', 'inspect']:
    if settings.get('container') is None:
        sys.exit(1)
    print(json.dumps([settings['container']]))
elif args[:2] == ['image', 'inspect']:
    sys.exit(0 if settings.get('image_exists', True) else 1)
elif args[0] == 'build':
    sys.exit(settings.get('build_exit', 0))
elif args[0] == 'start':
    sys.exit(settings.get('start_exit', 0))
'''
LAB = '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['LAUNCH_TEST_LOG'], 'a') as log:
    log.write(json.dumps(['lab', *sys.argv[1:]]) + '\\n')
'''


class StartLabTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='lab launcher ')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / 'ros_ws').mkdir()
        (self.root / 'scripts').mkdir()
        (self.root / 'bin').mkdir()
        shutil.copyfile(ROOT / 'start_lab.sh', self.root / 'start_lab.sh')
        for path, content in ((self.root / 'bin/docker', DOCKER),
                              (self.root / 'scripts/test_lab.sh', LAB),
                              (self.root / 'bin/npm', '#!/bin/sh\nexit 0\n'),
                              (self.root / 'bin/git', '#!/bin/sh\nexit 0\n')):
            path.write_text(content)
            path.chmod(0o755)
        self.log = self.root / 'calls.jsonl'
        self.settings = self.root / 'settings.json'
        self.environment = dict(os.environ, PATH=f"{self.root / 'bin'}:{os.environ['PATH']}",
                                LAUNCH_TEST_LOG=str(self.log), LAUNCH_TEST_SETTINGS=str(self.settings),
                                SIM_TEST_CONTAINER='fixture', SIM_TEST_IMAGE='fixture:latest',
                                DISPLAY='', XAUTHORITY='')

    def container(self, running=True):
        return {'HostConfig': {'NetworkMode': 'host'}, 'State': {'Running': running},
                'Mounts': [{'Type': 'bind', 'Source': str(self.root / 'ros_ws'),
                            'Destination': '/ros2_ws', 'RW': True}]}

    def launch(self, settings, *args):
        self.settings.write_text(json.dumps(settings))
        self.log.write_text('')
        result = subprocess.run(['bash', str(self.root / 'start_lab.sh'), *args], cwd='/tmp',
                                env=self.environment, text=True, capture_output=True, timeout=10)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        return result, calls

    def test_creates_detached_container_then_launches_lab_from_any_directory(self):
        result, calls = self.launch({})
        self.assertEqual(result.returncode, 0, result.stderr)
        run = next(call for call in calls if call[:2] == ['docker', 'run'])
        self.assertIn('--detach', run)
        self.assertIn('--init', run)
        self.assertEqual(run[run.index('--network') + 1], 'host')
        self.assertIn(f'type=bind,source={self.root / "ros_ws"},target=/ros2_ws', run)
        self.assertIn('GAZEBO_MODEL_PATH=/ros2_ws/models', run)
        self.assertEqual(run[-3:], ['fixture:latest', 'sleep', 'infinity'])
        self.assertEqual(calls[-2:], [['docker', 'exec', 'fixture', 'true'], ['lab']])

    def test_builds_missing_image_before_creating_container(self):
        result, calls = self.launch({'image_exists': False}, 'setup')
        self.assertEqual(result.returncode, 0, result.stderr)
        build = ['docker', 'build', '--tag', 'fixture:latest', str(self.root)]
        self.assertIn(build, calls)
        self.assertLess(calls.index(build), next(i for i, call in enumerate(calls) if call[:2] == ['docker', 'run']))
        self.assertEqual(calls[-1], ['lab', 'setup'])

    def test_reuses_running_container_without_recreating_or_building(self):
        result, calls = self.launch({'container': self.container()}, 'serve')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(any(call[1] in ('run', 'start', 'build') for call in calls if call[0] == 'docker'))
        self.assertEqual(calls[-1], ['lab', 'serve'])

    def test_starts_stopped_container_and_preserves_batch_path(self):
        batch = self.root / 'batch request.json'
        batch.write_text('{}')
        result, calls = self.launch({'container': self.container(running=False)}, 'run', str(batch))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(['docker', 'start', 'fixture'], calls)
        self.assertEqual(calls[-1], ['lab', 'run', str(batch)])

    def test_rejects_incompatible_containers_before_starting_or_launching(self):
        for mismatch in ('network', 'checkout', 'readonly'):
            with self.subTest(mismatch=mismatch):
                container = self.container(running=False)
                if mismatch == 'network':
                    container['HostConfig']['NetworkMode'] = 'bridge'
                elif mismatch == 'checkout':
                    container['Mounts'][0]['Source'] = '/tmp/another-checkout'
                else:
                    container['Mounts'][0]['RW'] = False
                result, calls = self.launch({'container': container})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('SIM_TEST_CONTAINER', result.stderr)
                self.assertFalse(any(call[0] == 'lab' or call[:2] == ['docker', 'start'] for call in calls))

    def test_build_or_start_failure_does_not_launch_lab(self):
        for settings in ({'image_exists': False, 'build_exit': 1},
                         {'container': self.container(running=False), 'start_exit': 1}):
            with self.subTest(settings=settings):
                result, calls = self.launch(settings)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(any(call[0] == 'lab' for call in calls))


if __name__ == '__main__':
    unittest.main()
