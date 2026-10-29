"""One isolated ROS experiment, with a recorder subscribed before launch."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback

from .metrics import summarize
from .processes import descendants, process_identity, terminate_launch
from .storage import TrajectoryWriter, atomic_json, utcnow
from .telemetry import Recording


def launch_arguments(run_dir):
    config = Path(run_dir) / 'config'
    return ['ros2', 'launch', 'sjtu_drone_bringup', 'bringup.launch.py', 'unattended:=true', 'live_visualizations:=true',
            f'simulation_config_file:={config / "simulation.xml"}',
            *[f'{argument}:={config / filename}' for argument, filename in (
                ('cow_config_file', 'cow_config.xml'), ('drone_control_config_file', 'drone_control_config.xml'),
                ('grid_config_file', 'grid_config.xml'), ('yolo_config_file', 'yolo_config.xml'),
                ('vehicle_config_file', 'drone.yaml'))]]


def execute(run_dir):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseArray
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Empty, String

    run_dir = Path(run_dir).resolve()
    manifest = json.loads((run_dir / 'manifest.json').read_text())
    writer = TrajectoryWriter(run_dir / 'trajectory.sqlite3')
    recording = Recording(manifest, writer)
    cancelled = [False]
    signal.signal(signal.SIGTERM, lambda *_: cancelled.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: cancelled.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('experiment_recorder')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    stop = node.create_publisher(Empty, '/simulation/stop', qos)
    process = None
    display = None
    views = None
    log = (run_dir / 'launch.log').open('w')
    started = utcnow()
    lease = {'worker': process_identity(os.getpid()), 'launch': None}
    atomic_json(run_dir / 'process.json', lease)
    expected = {e['name']: e['kind'] for e in manifest['entities']}

    def models(message):
        entries, seen = [], set()
        for index, (name, pose) in enumerate(zip(message.name, message.pose)):
            if name not in expected:
                continue
            if name in seen:
                recording.finish('failed', f'Duplicate model name: {name}')
                return
            seen.add(name)
            velocity = message.twist[index].linear if index < len(message.twist) else None
            entries.append({'name': name, 'kind': expected[name],
                            'pose': [pose.position.x, pose.position.y, pose.position.z,
                                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                            'velocity': [velocity.x, velocity.y, velocity.z] if velocity else [0, 0, 0]})
        recording.update_positions(entries)

    subscriptions = [
        node.create_subscription(Clock, '/clock',
                                 lambda m: recording.update_clock(m.clock.sec + m.clock.nanosec * 1e-9),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(ModelStates, '/gazebo/model_states', models, 100),
        node.create_subscription(PoseArray, manifest['cow_positions_topic'],
                                 lambda m: recording.update_detections([(p.position.x, p.position.y) for p in m.poses]), 50),
        node.create_subscription(Empty, '/simulation/stop',
                                 lambda m: setattr(recording, 'received_stop', True), qos)]
    for entity in manifest['entities']:
        if entity['kind'] == 'drone':
            subscriptions.append(node.create_subscription(
                String, entity['name'] + '/search_state',
                lambda message, name=entity['name']: recording.update_search(name, message.data), qos))
    try:
        # Refuse to attach a run to an already occupied ROS domain.
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not cancelled[0]:
            rclpy.spin_once(node, timeout_sec=.05)
        if recording.clock is not None:
            raise RuntimeError('The selected ROS domain already has a simulation clock')
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            master_port = reservation.getsockname()[1]
        environment = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{master_port}',
                           ROS_LOG_DIR=str(run_dir / 'ros_logs'), QT_QPA_PLATFORM='xcb')
        from .configuration import sha256
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        import importlib.util
        inputs = []
        for module in ('cow_pkg.repeller_node', 'cow_pkg.motion', 'sjtu_drone_control.drone_position_control',
                       'yolo_pkg.yolo_subscriber', 'yolo_pkg.cow_tracker', 'yolo_pkg.cow_tracker_with_kalman_filter'):
            spec = importlib.util.find_spec(module)
            if spec and spec.origin:
                inputs.append(Path(spec.origin))
        inputs += [Path(get_package_prefix('grid_visualizer_pkg')) / 'lib/grid_visualizer_pkg/grid_visualizer_node',
                   Path(get_package_share_directory('sjtu_drone_bringup')) / 'launch/bringup.launch.py',
                   Path(get_package_share_directory('sjtu_drone_bringup')) / 'launch/bringup_gazebo.launch.py',
                   Path(get_package_share_directory('sjtu_drone_description')) / 'urdf/sjtu_drone.urdf.xacro',
                   Path(get_package_share_directory('cow_pkg')) / 'models/turtlebot3_burger/model.sdf']
        weights = Path(manifest['detector_weights']['path'])
        if weights.is_file():
            inputs.append(weights)
        atomic_json(run_dir / 'runtime.json', {'ros_domain_id': environment.get('ROS_DOMAIN_ID'),
                                             'gazebo_master_uri': environment['GAZEBO_MASTER_URI'],
                                             'started_at': started,
                                             'installed_input_sha256': {str(p): sha256(p) for p in inputs if p.is_file()}})
        if not environment.get('DISPLAY'):
            import shutil
            if not shutil.which('Xvfb'):
                raise RuntimeError('Camera rendering needs DISPLAY or the Xvfb package')
            for number in range(90, 190):
                if not Path(f'/tmp/.X11-unix/X{number}').exists() and not Path(f'/tmp/.X{number}-lock').exists():
                    display = subprocess.Popen(['Xvfb', f':{number}', '-screen', '0', '1280x800x24',
                                                '-nolisten', 'tcp', '+extension', 'GLX'],
                                               stdout=log, stderr=subprocess.STDOUT)
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline and display.poll() is None:
                        if Path(f'/tmp/.X11-unix/X{number}').exists():
                            break
                        time.sleep(.05)
                    if display.poll() is None and Path(f'/tmp/.X11-unix/X{number}').exists():
                        environment['DISPLAY'] = f':{number}'
                        lease['display'] = process_identity(display.pid)
                        break
                    if display.poll() is None:
                        display.terminate()
                        display.wait(timeout=3)
            else:
                raise RuntimeError('Could not start a virtual display for camera rendering')
        process = subprocess.Popen(launch_arguments(run_dir), env=environment, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        from .live_views import LiveViews
        views = LiveViews(run_dir, manifest, node, environment, recording, virtual_display=display is not None)
        lease['launch'] = process_identity(process.pid)
        atomic_json(run_dir / 'process.json', lease)
        next_flush = next_lease = time.monotonic()
        while not recording.outcome:
            if cancelled[0]:
                recording.finish('cancelled', 'Cancelled by the operator')
                break
            if os.getppid() == 1:
                recording.finish('interrupted', 'Experiment supervisor exited')
                break
            if process.poll() is not None:
                recording.finish('failed', f'Simulation launch exited unexpectedly ({process.returncode}); see launch log')
                break
            rclpy.spin_once(node, timeout_sec=.05)
            recording.check_watchdogs()
            if not recording.outcome:
                views.tick()
            now = time.monotonic()
            if now >= next_flush and not recording.outcome:
                writer.flush()
                atomic_json(run_dir / 'state.json', dict(recording.state(), started_at=started))
                next_flush = now + .5
            if now >= next_lease:
                lease['descendants'] = descendants(process.pid)
                lease['auxiliary'] = views.identities()
                atomic_json(run_dir / 'process.json', lease)
                next_lease = now + 2
    except BaseException as error:
        traceback.print_exc(file=log)
        recording.finish('failed', str(error))
    finally:
        if not recording.outcome:
            recording.finish('interrupted', 'Recorder stopped unexpectedly')
        result = dict(recording.state(), started_at=started, finished_at=utcnow())
        # Stop physics and controllers before bounded process cleanup. Exclude
        # cleanup duration and braking samples from the experiment metrics.
        stop.publish(Empty())
        time.sleep(.25)
        if views:
            views.close()
        if process:
            terminate_launch(process)
        if display and display.poll() is None:
            display.terminate()
            try:
                display.wait(timeout=3)
            except subprocess.TimeoutExpired:
                display.kill()
                display.wait(timeout=3)
        writer.close()
        result['summary'] = summarize(run_dir / 'trajectory.sqlite3', manifest['scenario'], list(expected))
        atomic_json(run_dir / 'state.json', result)
        atomic_json(run_dir / 'summary.json', result['summary'])
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    execute(parser.parse_args().run_dir)


if __name__ == '__main__':
    main()
