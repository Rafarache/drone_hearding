"""ROS integration checks; run after building and sourcing the workspace."""

import math
import os
from pathlib import Path
import re
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseArray, Twist, Vector3
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from cow_pkg.repeller_node import PeopleRepeller
from cow_pkg.simulation_completion_monitor import SimulationCompletionMonitor
from sjtu_drone_control.drone_position_control import DronePositionControl
from sjtu_drone_control.drone_utils.drone_object import DroneObject
from sjtu_drone_control.teleop import TeleopNode as Keyboard
from sjtu_drone_control.teleop_joystick import TeleopNode as Joystick
from yolo_pkg.search_observation import ObservationSampler


def pose(x, y):
    result = Pose()
    result.position.x, result.position.y = float(x), float(y)
    result.orientation.w = 1.0
    return result


def spin(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def stop_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def test_completion_stops_controllers_and_late_subscribers():
    rclpy.init(args=['--ros-args', '-p', 'number_of_cows:=2', '-p', 'number_of_drones:=1',
                     '-p', 'drone_namespace_prefix:=/test_drone', '-p', 'cow_name_prefix:=cow',
                     '-p', 'drone_approach_radius:=2.0', '-p', 'push_point_margin:=2.0',
                     '-p', 'drone_namespace:=/test_drone0', '-p', 'goal_x:=0.0', '-p', 'goal_y:=0.0'])
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        driver = Node('mission_test_driver')
        monitor, repeller, position = SimulationCompletionMonitor(), PeopleRepeller(), DronePositionControl()
        keyboard, joystick, base = Keyboard(), Joystick(), DroneObject('legacy_test')
        nodes.extend([driver, monitor, repeller, position, keyboard, joystick, base])
        for node in nodes:
            executor.add_node(node)
        commands, cow_commands, stops = [], [], []
        subscriptions = [
            driver.create_subscription(Twist, '/test_drone0/cmd_vel', commands.append, 10),
            driver.create_subscription(Twist, '/cow0/cmd_vel', cow_commands.append, 10),
            driver.create_subscription(Empty, '/simulation/stop', stops.append, stop_qos()),
        ]
        states = driver.create_publisher(ModelStates, '/gazebo/model_states', 10)
        spin(executor, 0.6)
        incomplete = ModelStates(name=['cow0'], pose=[pose(0, 0)])
        states.publish(incomplete)
        spin(executor, 0.3)
        assert not stops
        outside = ModelStates(name=['cow0', 'cow1'], pose=[pose(0, 0), pose(2.01, 0)])
        states.publish(outside)
        spin(executor, 0.3)
        assert not stops
        states.publish(ModelStates(name=['cow0', 'cow1'], pose=[pose(0, 0), pose(2, 0)]))
        spin(executor, 0.6)
        assert len(stops) == 1 and repeller.stopped and position.stop_latch.stopped
        assert keyboard.stop_latch.stopped and joystick.stop_latch.stopped and base.stop_latch.stopped
        commands.clear()
        cow_commands.clear()
        position.goto_callback(pose(20, 20))
        position.focusin_callback(pose(20, 20))
        keyboard.publish_cmd_vel(Vector3(x=1.0))
        joystick.joy_callback(Joy(axes=[1.0] * 6, buttons=[1] * 2))
        assert base.move(Vector3(x=1.0)) is False
        states.publish(outside)  # completion cannot resume after cows drift
        spin(executor, 0.4)
        assert commands and cow_commands
        assert all(msg == Twist() for msg in commands + cow_commands)
        late_stops = []
        subscriptions.append(driver.create_subscription(Empty, '/simulation/stop', late_stops.append, stop_qos()))
        late_position = DronePositionControl()
        nodes.append(late_position)
        executor.add_node(late_position)
        spin(executor, 0.5)
        assert late_stops and late_position.stop_latch.stopped and not late_position.takeoff_sent
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_live_coverage_search_herding_and_stop(tmp_path):
    rclpy.init()
    driver = Node('field_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) / 'lib/grid_visualizer_pkg/grid_visualizer_node')
    processes, files = [], []
    try:
        for index in (0, 1):
            output = (tmp_path / f'grid{index}.log').open('w')
            files.append(output)
            parameters = dict(drone_index=index, total_drones=2, global_map_center_x=0.0,
                              global_map_center_y=0.0, global_map_width=20.0, global_map_height=20.0,
                              drone_namespace_prefix='/field_drone', cow_positions_topic='/field_cows',
                              cow_exclusion_radius=2.0, push_point_margin=2.0, goal_x=-8.0, goal_y=-8.0,
                              show_vector_grid=False, show_occupancy_grid=False, show_coverage_map=False)
            args = [executable, '--ros-args', '-r', f'__node:=test_grid{index}']
            for key, value in parameters.items():
                args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
            processes.append(subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT))
        updates, targets, focuses = [[], []], [[], []], [[], []]
        subscriptions, poses, observations = [], [], []
        for i in (0, 1):
            prefix = f'/field_drone{i}'
            subscriptions.extend([
                driver.create_subscription(PolygonStamped, prefix + '/searched_area', updates[i].append, 50),
                driver.create_subscription(Pose, prefix + '/goto', targets[i].append, 10),
                driver.create_subscription(Pose, prefix + '/focusin', focuses[i].append, 10),
            ])
            poses.append(driver.create_publisher(Pose, prefix + '/gt_pose', 10))
            observations.append(driver.create_publisher(PolygonStamped, prefix + '/processed_view', 50))
        cows = driver.create_publisher(PoseArray, '/field_cows', 10)
        stopper = driver.create_publisher(Empty, '/simulation/stop', stop_qos())
        spin(executor, 0.8)
        for i in (0, 1):
            poses[i].publish(pose(0, i * 4))
        spin(executor, 0.6)
        assert targets[0] and all(p.position.x == 0 and p.position.y == 0 for p in targets[0])
        # Complete the initial outward turn. This fixture supplies poses itself
        # and has no flight controller to execute focus commands.
        for i, yaw in enumerate((-math.pi / 2, math.pi / 2)):
            aligned = pose(0, i * 4)
            aligned.orientation.z = math.sin(yaw / 2)
            aligned.orientation.w = math.cos(yaw / 2)
            poses[i].publish(aligned)
        spin(executor, 0.6)
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.header.stamp = driver.get_clock().now().to_msg()
        footprint.polygon.points = [Point32(x=x, y=y, z=0.0)
                                    for x, y in ObservationSampler().sample('a', 0, 0, 0, 0)]
        assert len(footprint.polygon.points) == 4
        observations[0].publish(footprint)
        spin(executor, 1.0)
        assert len(updates[0]) == 1 and len(updates[1]) == 0  # no peer echo
        # Compare wire values: Point32 rounds Python doubles during serialization.
        assert serialize_message(updates[0][0]) == serialize_message(footprint)
        assert any(abs(p.position.x) + abs(p.position.y) > 0.1 for p in targets[0])
        assert focuses[0]  # camera commands exist without any cows
        observations[1].publish(footprint)
        cows.publish(PoseArray(poses=[pose(5, 0)]))
        spin(executor, 5.0)
        for output in files:
            output.flush()
        logs = [(tmp_path / f'grid{i}.log').read_text() for i in (0, 1)]
        assert all(re.search(r'objectives=[1-9][0-9]*/1', log) for log in logs), logs
        assert all(values for values in focuses)
        coverage = [re.findall(r'coverage=([0-9.]+)%', log)[-1] for log in logs]
        assert coverage[0] == coverage[1]
        # Tracker-empty updates retire cow objectives but must not stop search.
        count = len(targets[0])
        cows.publish(PoseArray())
        spin(executor, 0.7)
        assert len(targets[0]) > count
        stopper.publish(Empty())
        spin(executor, 0.4)
        lengths = [len(t) for t in targets]
        for i in (0, 1):
            poses[i].publish(pose(1, 1))
            observations[i].publish(footprint)
        cows.publish(PoseArray(poses=[pose(6, 6)]))
        spin(executor, 0.7)
        assert lengths == [len(t) for t in targets]
        assert all(process.poll() is None for process in processes)
    finally:
        for process in processes:
            process.terminate()
            process.wait(timeout=5)
        for output in files:
            output.close()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_remembered_cow_drives_intermediate_targets_after_drone_moves_away(tmp_path):
    from yolo_pkg.cow_tracker import CowTrackerManager

    rclpy.init()
    driver = Node('cow_memory_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) / 'lib/grid_visualizer_pkg/grid_visualizer_node')
    output = (tmp_path / 'memory_grid.log').open('w')
    parameters = dict(drone_index=0, total_drones=1, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=100.0, global_map_height=100.0,
                      drone_namespace_prefix='/memory_drone', cow_positions_topic='/memory_cows',
                      cow_exclusion_radius=2.0, push_point_margin=2.0, goal_x=-40.0, goal_y=0.0,
                      show_vector_grid=False, show_occupancy_grid=False, show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=memory_grid0']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        targets = []
        subscription = driver.create_subscription(Pose, '/memory_drone0/goto', targets.append, 10)
        position = driver.create_publisher(Pose, '/memory_drone0/gt_pose', 10)
        views = driver.create_publisher(PolygonStamped, '/memory_drone0/processed_view', 50)
        cows = driver.create_publisher(PoseArray, '/memory_cows', 10)
        tracker = CowTrackerManager(max_lost_frames=3, max_position_distance=3.0,
                                    min_hits=1, reacquisition_distance=5.0)
        camera = dict(x=20.0, y=10.0, yaw=0.0, horizontal_fov=2.09, max_range=20.0)
        tracks = tracker.update([(25.0, 10.0)], timestamp=0.0, camera_views=[camera])
        cow_id = tracks[0]['id']
        spin(executor, 0.8)
        # Synthetic full-field coverage isolates cow navigation from search.
        # Cow memory and local intermediate targets are the regression here.
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in
                                    [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]]
        views.publish(footprint)
        position.publish(pose(20, 10))
        cows.publish(PoseArray(poses=[pose(t['x'], t['y']) for t in tracks]))
        spin(executor, 0.7)
        assert targets and process.poll() is None

        # Visible misses beyond the old expiry limit cannot empty the cow map.
        for step in range(1, 31):
            tracks = tracker.update([], timestamp=step * 0.1, camera_views=[camera])
        assert [(t['id'], t['x'], t['y']) for t in tracks] == [(cow_id, 25.0, 10.0)]
        timestamp = 3.0
        for x, y in [(-20.0, -10.0), (-30.0, 20.0)]:
            position.publish(pose(x, y))
            distant_camera = dict(camera, x=x, y=y)
            for _ in range(30):
                timestamp += 0.1
                tracks = tracker.update([], timestamp=timestamp, camera_views=[distant_camera])
            assert [(t['id'], t['x'], t['y']) for t in tracks] == [(cow_id, 25.0, 10.0)]
            cows.publish(PoseArray(poses=[pose(t['x'], t['y']) for t in tracks]))
            spin(executor, 0.3)
            targets.clear()
            spin(executor, 0.8)
            assert targets and process.poll() is None
            assert any(p.position.x > x + 0.2 for p in targets)
            if y < 10:
                assert any(p.position.y > y + 0.01 for p in targets)
            else:
                assert any(p.position.y < y - 0.01 for p in targets)
            assert all(abs(p.position.x - x) <= 0.51 and abs(p.position.y - y) <= 0.51
                       for p in targets)  # local steps, never a jump to the far cow
        position.publish(pose(20, 10))  # revisiting keeps the same real cow entry
        tracks = tracker.update([(25.2, 10.0)], timestamp=timestamp + 0.1, camera_views=[camera])
        assert len(tracks) == 1 and tracks[0]['id'] == cow_id and not tracks[0]['coasting']
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_search_viewpoints_follow_approach_and_remain_until_observed(tmp_path):
    rclpy.init()
    driver = Node('approach_view_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    log_path = tmp_path / 'approach_grid.log'
    output = log_path.open('w')
    parameters = dict(drone_index=0, total_drones=1, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=60.0, global_map_height=60.0,
                      drone_namespace_prefix='/approach_drone', cow_positions_topic='/approach_cows',
                      cow_exclusion_radius=2.0, push_point_margin=2.0, goal_x=-10.0, goal_y=-10.0,
                      show_vector_grid=False, show_occupancy_grid=False, show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=approach_grid0']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        position = driver.create_publisher(Pose, '/approach_drone0/gt_pose', 10)
        observations = driver.create_publisher(PolygonStamped, '/approach_drone0/processed_view', 50)
        cows = driver.create_publisher(PoseArray, '/approach_cows', 10)
        targets = []
        subscription = driver.create_subscription(Pose, '/approach_drone0/goto', targets.append, 10)
        focuses = []
        focus_subscription = driver.create_subscription(
            Pose, '/approach_drone0/focusin', focuses.append, 10)
        spin(executor, 0.8)
        position.publish(pose(24, 11))

        def observe_rectangle(left, bottom, right, top):
            footprint = PolygonStamped()
            footprint.header.frame_id = 'map'
            footprint.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in
                                        [(left, bottom), (right, bottom), (right, top), (left, top)]]
            observations.publish(footprint)

        # Leave two whole 2 m patches unseen, centered at (+/-11, 11).
        for rectangle in [(-30, -30, 30, 10), (-30, 12, 30, 30),
                          (-30, 10, -12, 12), (-10, 10, 10, 12), (12, 10, 30, 12)]:
            observe_rectangle(*rectangle)
        spin(executor, 1.8)
        first = re.findall(r'search viewpoint: patch=\(11.00,11.00\) '
                           r'attraction=\(([-\d.]+),([-\d.]+)\)', log_path.read_text())
        assert first, log_path.read_text()
        assert 11 < float(first[-1][0]) < 24 and abs(float(first[-1][1])-11) < .01
        # The attraction is about 6.9 m away, the cow centroid is 9 m away,
        # and the patch is 13 m away. Compare the attraction, then FACE the patch.
        cows.publish(PoseArray(poses=[pose(15, 9), pose(15, 13)]))
        spin(executor, 1.0)
        assert focuses and abs(focuses[-1].position.x-11) < 1e-6
        assert abs(focuses[-1].position.y-11) < 1e-6
        # The cow centroid moves closer (4 m), without blocking the attraction.
        cows.publish(PoseArray(poses=[pose(20, 9), pose(20, 13)]))
        spin(executor, 1.0)
        assert abs(focuses[-1].position.x-20) < 1e-6
        assert abs(focuses[-1].position.y-11) < 1e-6
        # With no eligible cows, resume search attention.
        cows.publish(PoseArray())
        spin(executor, 1.0)
        assert abs(focuses[-1].position.x-11) < 1e-6
        # Movement alone cannot relocate the selected attraction or complete coverage.
        position.publish(pose(-24, 11))
        spin(executor, 1.3)
        assert not re.search(r'search viewpoint: patch=\(-11.00,11.00\)', log_path.read_text())
        again = re.findall(r'search viewpoint: patch=\(11.00,11.00\)', log_path.read_text())
        assert len(again) == len(first)
        # Completion selects the remaining patch using the new drone position.
        observe_rectangle(10, 10, 12, 12)
        spin(executor, 1.8)
        second = re.findall(r'search viewpoint: patch=\(-11.00,11.00\) '
                            r'attraction=\(([-\d.]+),([-\d.]+)\)', log_path.read_text())
        assert second, log_path.read_text()
        assert -24 < float(second[-1][0]) < -11 and abs(float(second[-1][1])-11) < .01
        observe_rectangle(-12, 10, -10, 12)
        spin(executor, 1.8)
        assert targets and targets[-1].position.x == -24 and targets[-1].position.y == 11
        # No search remains: eligible local cows still receive attention.
        cows.publish(PoseArray(poses=[pose(-20, 9), pose(-20, 13)]))
        spin(executor, 1.0)
        assert abs(focuses[-1].position.x+20) < 1e-6
        assert abs(focuses[-1].position.y-11) < 1e-6
        # Neither target exists: preserve the drone's current heading (+X).
        cows.publish(PoseArray())
        spin(executor, 1.0)
        assert abs(focuses[-1].position.x+14) < 1e-6
        assert abs(focuses[-1].position.y-11) < 1e-6
        assert process.poll() is None and subscription and focus_subscription
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_cow_push_waypoint_is_not_a_terminal_hold(tmp_path):
    rclpy.init()
    driver = Node('cow_waypoint_test_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    log_path = tmp_path / 'cow_waypoint.log'
    output = log_path.open('w')
    parameters = dict(drone_index=0, total_drones=1, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=40.0, global_map_height=40.0,
                      drone_namespace_prefix='/cow_waypoint_drone',
                      cow_positions_topic='/cow_waypoint_cows', cow_exclusion_radius=2.0,
                      push_point_margin=2.0, goal_x=-10.0, goal_y=0.0,
                      show_vector_grid=False, show_occupancy_grid=False, show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=cow_waypoint_grid']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        positions = driver.create_publisher(Pose, '/cow_waypoint_drone0/gt_pose', 10)
        observations = driver.create_publisher(
            PolygonStamped, '/cow_waypoint_drone0/processed_view', 10)
        cows = driver.create_publisher(PoseArray, '/cow_waypoint_cows', 10)
        targets = []
        subscription = driver.create_subscription(
            Pose, '/cow_waypoint_drone0/goto', targets.append, 10)
        spin(executor, 0.8)
        # Full coverage isolates herding. Estimated cow x=6 gives a pushing
        # waypoint x=10. The drone is already within the former 0.5 m hold zone.
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in
                                    [(-20.0, -20.0), (20.0, -20.0),
                                     (20.0, 20.0), (-20.0, 20.0)]]
        observations.publish(footprint)
        cows.publish(PoseArray(poses=[pose(6, 0)]))
        positions.publish(pose(10.2, 0))
        spin(executor, 1.5)
        targets.clear()
        spin(executor, 1.0)
        assert process.poll() is None and targets, log_path.read_text()
        assert any(p.position.x < 10.1 for p in targets), log_path.read_text()
        assert all(math.hypot(p.position.x-10.2, p.position.y) <= .50001 for p in targets)
        assert subscription
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
