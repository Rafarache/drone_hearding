"""Real ROS message-to-navigation checks in a separate ROS_DOMAIN_ID."""

import math
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from cow_mapping_interfaces.msg import CowDetection, CowObservation
from geometry_msgs.msg import Point32, PolygonStamped, Pose
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def pose(x, y, yaw=0):
    result = Pose()
    result.position.x, result.position.y = float(x), float(y)
    result.orientation.z, result.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return result


def test_raw_evidence_controls_navigation_and_resets_on_clock_jump(tmp_path):
    rclpy.init()
    driver = Node('evidence_ros_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    log_path = tmp_path / 'evidence.log'
    output = log_path.open('w')
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    parameters = dict(drone_index=0, total_drones=2, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=40.0, global_map_height=40.0,
                      drone_namespace_prefix='/evidence_drone', cow_observations_topic='/evidence',
                      cow_exclusion_radius=2.0, push_point_margin=2.0, goal_x=-10.0, goal_y=0.0,
                      use_sim_time=True, show_vector_grid=False, show_occupancy_grid=False,
                      show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=evidence_grid']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        own_pose = driver.create_publisher(Pose, '/evidence_drone0/gt_pose', 10)
        peer_pose = driver.create_publisher(Pose, '/evidence_drone1/gt_pose', 10)
        coverage = driver.create_publisher(PolygonStamped, '/evidence_drone0/processed_view', 10)
        observations = driver.create_publisher(CowObservation, '/evidence', 50)
        clocks = driver.create_publisher(Clock, '/clock', 10)
        focuses = []
        subscription = driver.create_subscription(Pose, '/evidence_drone0/focusin', focuses.append, 10)
        clock = Clock()
        clock.clock.sec = 100

        def spin(seconds=.65):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                clocks.publish(clock)
                own_pose.publish(pose(0, 0, -math.pi/2))
                peer_pose.publish(pose(0, 15))
                executor.spin_once(timeout_sec=.02)
            assert process.poll() is None, log_path.read_text()

        def facing_cow():
            assert focuses, log_path.read_text()
            return math.hypot(focuses[-1].position.x-5.25, focuses[-1].position.y-.25) < .01

        stamp = 0

        def send(points=(), source=0, yaw=0, reuse=None, frame_id='map'):
            nonlocal stamp
            stamp += 1
            obs = CowObservation()
            obs.header.frame_id = frame_id
            obs.header.stamp.sec = stamp if reuse is None else reuse
            obs.source_drone_index = source
            obs.camera_pose = pose(0, 0, yaw)
            obs.near_range_m, obs.far_range_m, obs.horizontal_fov_rad = 2.0, 20.0, 2.09
            for x, y in points:
                detection = CowDetection()
                detection.position.x, detection.position.y = float(x), float(y)
                detection.position_valid = True
                detection.bearing_min_rad, detection.bearing_max_rad = -.1, .1
                obs.detections.append(detection)
            observations.publish(obs)
            spin()
            return obs.header.stamp.sec

        spin(1.0)
        full = PolygonStamped()
        full.header.frame_id = 'map'
        full.polygon.points = [Point32(x=x,y=y,z=0.0) for x,y in
                               [(-20.0,-20.0),(20.0,-20.0),(20.0,20.0),(-20.0,20.0)]]
        coverage.publish(full)
        spin()
        first_stamp = send([(5,0)])
        assert not facing_cow()  # score 150
        send([(5,0)], reuse=first_stamp)
        assert not facing_cow()  # duplicate does not confirm a cow
        send([(5,0)])
        assert facing_cow()  # second unique hit -> 255, quantized center
        for _ in range(3):
            send(source=1)
        assert facing_cow()  # peer empty views cannot clear own map
        send(); assert facing_cow()  # 225
        send(); assert facing_cow()  # 195
        send(); assert not facing_cow()  # 165
        send([(5,0)], reuse=first_stamp)
        assert not facing_cow()  # out-of-order hit ignored
        send([(5,0)], source=1)
        assert facing_cow()  # peer positive evidence participates
        for _ in range(3):
            send(yaw=math.pi)
        assert facing_cow()  # empty observations facing away cannot clear it
        clock.clock.sec = 1
        spin()
        assert not facing_cow()  # simulation reset clears scores
        send([(5,0)], reuse=1)
        assert not facing_cow()
        send([(5,0)], reuse=2)
        assert facing_cow()  # per-source timestamp gate also reset
        assert subscription
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
