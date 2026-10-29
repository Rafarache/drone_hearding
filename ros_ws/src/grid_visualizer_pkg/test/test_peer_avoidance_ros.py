"""Headless peer obstacle regression; run in an isolated ROS domain."""

import math
from pathlib import Path
import subprocess
import time
import resource
import xml.etree.ElementTree as ET

import pytest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def pose(x, y):
    result = Pose()
    result.position.x, result.position.y = float(x), float(y)
    result.position.z, result.orientation.w = 1.0, 1.0
    return result


def test_circular_one_cell_peer_obstacle_moves_with_peer(tmp_path):
    rclpy.init()
    driver = Node('peer_obstacle_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    log_path = tmp_path / 'peer_obstacle.log'
    output = log_path.open('w')
    parameters = dict(drone_index=0, total_drones=2, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=40.0,
                      global_map_height=40.0, drone_namespace_prefix='/peer_test_drone',
                      cow_positions_topic='/peer_test_cows', cow_exclusion_radius=2.0,
                      push_point_margin=2.0, goal_x=-15.0, goal_y=0.0,
                      show_vector_grid=False, show_occupancy_grid=False,
                      show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=peer_obstacle_test_grid']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        positions = driver.create_publisher(Pose, '/peer_test_drone0/gt_pose', 10)
        peers = driver.create_publisher(Pose, '/peer_test_drone1/gt_pose', 10)
        cows = driver.create_publisher(PoseArray, '/peer_test_cows', 10)
        observations = driver.create_publisher(PolygonStamped, '/peer_test_drone0/processed_view', 10)
        commands = []
        subscription = driver.create_subscription(Pose, '/peer_test_drone0/goto', commands.append, 50)
        own = pose(0, 0)
        # Already face away from the first peer to complete startup alignment.
        own.orientation.z, own.orientation.w = 1.0, 0.0
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in
                                    [(-20.0, -20.0), (20.0, -20.0),
                                     (20.0, 20.0), (-20.0, 20.0)]]

        def publish_for(peer, duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                positions.publish(own)
                peers.publish(pose(*peer))
                cows.publish(PoseArray(poses=[pose(0, 8)]))
                observations.publish(footprint)
                executor.spin_once(timeout_sec=0.03)
            assert process.poll() is None, log_path.read_text()

        def settled_commands(peer):
            publish_for(peer, 1.0)
            commands.clear()
            publish_for(peer, 1.1)
            assert commands, log_path.read_text()
            assert all(math.hypot(p.position.x, p.position.y) <= .50001 for p in commands)
            return list(commands)

        publish_for((5, 0), 1.5)
        assert any(math.hypot(p.position.x, p.position.y) > .1
                   for p in settled_commands((5, 0))), log_path.read_text()
        # A one-cell radius is 0.5 m: 0.9 m separation must allow navigation.
        assert any(math.hypot(p.position.x, p.position.y) > .1
                   for p in settled_commands((.9, 0))), log_path.read_text()
        # The peer's 0.5 m inflated circle covers our current grid cell.
        assert all(math.hypot(p.position.x, p.position.y) < 1e-6
                   for p in settled_commands((.4, 0))), log_path.read_text()
        # Outside the circular radius at a diagonal: do not use a square mask.
        diagonal = settled_commands((.45, .45))
        assert any(math.hypot(p.position.x, p.position.y) > .1 for p in diagonal), log_path.read_text()
        for command in diagonal:
            for step in range(101):
                x, y = command.position.x * step / 100, command.position.y * step / 100
                # Match the centers of the moving grid's 0.5 m cells.
                cx = (math.floor(x / .5) + .5) * .5
                cy = (math.ceil(y / .5) - .5) * .5
                assert math.hypot(cx - .45, cy - .45) > .5
        # A moving peer must remove its old exclusion on the next grid update.
        assert any(math.hypot(p.position.x, p.position.y) > .1
                   for p in settled_commands((5, 0))), log_path.read_text()
        assert subscription  # Keep the subscription alive until shutdown.
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.parametrize('key,value', [
    ('peer_drone_repulsion_potential', '2.0'),
    ('peer_drone_repulsion_potential', '0.8'),
    ('boundary_potential', '1.1'),
    ('free_cell_potential', '-0.1'),
    ('free_cell_potential', '1.1'),
])
def test_invalid_potential_configuration_is_rejected(tmp_path, key, value):
    config = ET.parse(str(Path(get_package_share_directory('grid_visualizer_pkg')) /
                          'config/grid_config.xml'))
    config.getroot().find(key).text = value
    path = tmp_path / 'invalid_grid.xml'
    config.write(path, encoding='utf-8', xml_declaration=True)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    result = subprocess.run([executable, '--ros-args', '-p', f'config_file:={path}',
                             '-p', 'show_vector_grid:=false', '-p', 'show_occupancy_grid:=false',
                             '-p', 'show_coverage_map:=false'],
                            capture_output=True, text=True, timeout=5,
                            preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE, (0, 0)))
    assert result.returncode != 0
    assert 'potential' in result.stderr, result.stderr
