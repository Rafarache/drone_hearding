"""Exercise the real flight plugin without cameras or a display server."""

import copy
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, Int8


def drone_model(name, x):
    description = Path(get_package_share_directory('sjtu_drone_description'))
    template = ET.parse(description / 'models/sjtu_drone/sjtu_drone.sdf')
    plugin = copy.deepcopy(template.find(".//plugin[@filename='libplugin_drone.so']"))
    plugin.find('ros/namespace').text = '/' + name
    # Nonzero noise must be bypassed after completion, including later commands.
    plugin.find('motionSmallNoise').text = '0.2'
    ET.SubElement(plugin, 'pub_odom').text = 'true'
    model = ET.fromstring(f'''
        <model name="{name}"><pose>{x} 0 0.12 0 0 0</pose>
          <link name="base_footprint">
            <inertial><mass>1.477</mass><inertia>
              <ixx>0.1152</ixx><iyy>0.1152</iyy><izz>0.218</izz>
            </inertia></inertial>
            <collision name="box"><geometry><box><size>0.5 0.5 0.2</size>
            </box></geometry></collision>
          </link>
        </model>''')
    model.append(plugin)
    return model


@pytest.mark.skipif(not shutil.which('gzserver'), reason='Gazebo Classic is required')
def test_plugin_stops_takeoff_landing_position_mode_and_late_spawn(tmp_path):
    world = ET.fromstring('''<sdf version="1.6"><world name="stop_test">
      <physics type="ode"><max_step_size>0.001</max_step_size>
        <real_time_update_rate>1000</real_time_update_rate></physics>
      <model name="floor"><static>true</static><link name="floor">
        <collision name="plane"><geometry><plane><normal>0 0 1</normal>
        <size>100 100</size></plane></geometry></collision>
      </link></model></world></sdf>''')
    names = ['stop_landed', 'stop_takeoff', 'stop_landing', 'stop_position']
    for index, name in enumerate(names):
        world.find('world').append(drone_model(name, index * 5))
    world_path = tmp_path / 'stop.world'
    ET.ElementTree(world).write(world_path)
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    environment = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}')
    plugin_dir = str(Path(get_package_prefix('sjtu_drone_description')) / 'lib')
    environment['GAZEBO_PLUGIN_PATH'] = plugin_dir + ':' + environment.get('GAZEBO_PLUGIN_PATH', '')
    output = (tmp_path / 'gzserver.log').open('w')
    server = subprocess.Popen(['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                               '-s', 'libgazebo_ros_factory.so', str(world_path)],
                              stdout=output, stderr=subprocess.STDOUT, env=environment)
    rclpy.init()
    driver = Node('gazebo_stop_test')
    states, poses, speeds = {}, {}, {}
    subscriptions = []
    publishers = {}
    for name in names + ['stop_late']:
        def remember(target, key):
            return lambda msg: target.__setitem__(key, msg)
        subscriptions.extend([
            driver.create_subscription(Int8, f'/{name}/state', remember(states, name), 10),
            driver.create_subscription(Pose, f'/{name}/gt_pose', remember(poses, name), 10),
            driver.create_subscription(Twist, f'/{name}/gt_vel', remember(speeds, name), 10),
        ])
        publishers[name] = {
            'takeoff': driver.create_publisher(Empty, f'/{name}/takeoff', 10),
            'land': driver.create_publisher(Empty, f'/{name}/land', 10),
            'reset': driver.create_publisher(Empty, f'/{name}/reset', 10),
            'posctrl': driver.create_publisher(Bool, f'/{name}/posctrl', 10),
            'vel_mode': driver.create_publisher(Bool, f'/{name}/vel_mode', 10),
            'cmd_vel': driver.create_publisher(Twist, f'/{name}/cmd_vel', 10),
        }
    stop = driver.create_publisher(Empty, '/simulation/stop',
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait_until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(driver, timeout_sec=0.01)
            if predicate():
                return
            assert server.poll() is None, (tmp_path / 'gzserver.log').read_text()
        raise AssertionError(f'Gazebo condition timed out: states={states}, poses={poses}')

    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(driver, timeout_sec=0.01)

    try:
        wait_until(lambda: all(name in states for name in names))
        for name in names[2:]:
            publishers[name]['takeoff'].publish(Empty())
        wait_until(lambda: all(states[name].data == 1 and poses[name].position.z > 0.2
                               for name in names[2:]))
        publishers['stop_position']['posctrl'].publish(Bool(data=True))
        publishers['stop_position']['cmd_vel'].publish(
            Twist(linear=Vector3(x=18.0, y=2.0, z=2.0)))
        pump(0.5)
        publishers['stop_landing']['land'].publish(Empty())
        publishers['stop_takeoff']['takeoff'].publish(Empty())
        wait_until(lambda: states['stop_landing'].data == 3 and states['stop_takeoff'].data == 2,
                   timeout=0.45)
        stop.publish(Empty())
        wait_until(lambda: states['stop_takeoff'].data == states['stop_landing'].data == 1)
        # Further commands, mode changes, reset, and takeoff cannot undo the latch.
        for controls in publishers.values():
            controls['reset'].publish(Empty())
            controls['posctrl'].publish(Bool(data=True))
            controls['vel_mode'].publish(Bool(data=False))
            controls['cmd_vel'].publish(Twist(linear=Vector3(x=30.0, y=20.0, z=8.0)))
            controls['takeoff'].publish(Empty())
            controls['land'].publish(Empty())
        pump(4.0)
        assert states['stop_landed'].data == 0
        assert all(states[name].data == 1 for name in names[1:])
        for name in names:
            velocity = speeds[name].linear
            assert math.hypot(velocity.x, velocity.y, velocity.z) < 0.12, (name, velocity)
        before = copy.deepcopy(poses)
        pump(1.0)
        assert all(math.hypot(poses[name].position.x - before[name].position.x,
                              poses[name].position.y - before[name].position.y,
                              poses[name].position.z - before[name].position.z) < 0.12
                   for name in names)
        spawn = driver.create_client(SpawnEntity, '/spawn_entity')
        assert spawn.wait_for_service(timeout_sec=5)
        model_sdf = ET.Element('sdf', version='1.6')
        model_sdf.append(drone_model('stop_late', 25))
        request = SpawnEntity.Request(name='stop_late', xml=ET.tostring(model_sdf, encoding='unicode'))
        future = spawn.call_async(request)
        wait_until(future.done)
        assert future.result().success, future.result().status_message
        wait_until(lambda: 'stop_late' in states)
        pump(0.5)  # allow DDS discovery and retained delivery to the new plugin
        publishers['stop_late']['takeoff'].publish(Empty())
        publishers['stop_late']['cmd_vel'].publish(Twist(linear=Vector3(z=5.0)))
        pump(1.0)
        assert states['stop_late'].data == 0 and poses['stop_late'].position.z < 0.2
        assert server.poll() is None  # completion leaves the simulator alive
    finally:
        driver.destroy_node()
        rclpy.shutdown()
        server.terminate()
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait(timeout=5)
        output.close()
