"""Cow-blocked pushing goals are suppressed and return when the area clears."""

from pathlib import Path
import re
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def pose(x, y):
    result = Pose()
    result.position.x, result.position.y = float(x), float(y)
    result.orientation.w = 1.0
    return result


def test_cow_blocked_goals_are_omitted_and_restored(tmp_path):
    rclpy.init()
    driver = Node('cow_goal_obstacle_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executable = str(Path(get_package_prefix('grid_visualizer_pkg')) /
                     'lib/grid_visualizer_pkg/grid_visualizer_node')
    log = tmp_path / 'cow_goals.log'
    output = log.open('w')
    parameters = dict(drone_index=0, total_drones=1, global_map_center_x=0.0,
                      global_map_center_y=0.0, global_map_width=40.0, global_map_height=40.0,
                      drone_namespace_prefix='/blocked_goal_drone',
                      cow_positions_topic='/blocked_goal_cows', cow_exclusion_radius=2.0,
                      push_point_margin=2.0, goal_x=0.0, goal_y=0.0, use_sim_time=True,
                      goal_radius=3.0, final_goal_reached_radius=5.0,
                      show_vector_grid=False, show_occupancy_grid=False, show_coverage_map=False)
    args = [executable, '--ros-args', '-r', '__node:=cow_goal_obstacle_grid']
    for key, value in parameters.items():
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)
    try:
        positions = driver.create_publisher(Pose, '/blocked_goal_drone0/gt_pose', 10)
        cows = driver.create_publisher(PoseArray, '/blocked_goal_cows', 10)
        observations = driver.create_publisher(PolygonStamped, '/blocked_goal_drone0/processed_view', 10)
        clock = driver.create_publisher(Clock, '/clock', 10)
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in
                                    [(-20.0, -20.0), (20.0, -20.0),
                                     (20.0, 20.0), (-20.0, 20.0)]]
        started = time.monotonic()

        def check(snapshot, expected):
            offset = len(log.read_text())
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                stamp = Clock()
                seconds = 100 + 20 * (time.monotonic() - started)
                stamp.clock.sec = int(seconds)
                stamp.clock.nanosec = int((seconds - int(seconds)) * 1e9)
                cows.publish(PoseArray(poses=[pose(*p) for p in snapshot]))
                positions.publish(pose(6, 4))
                observations.publish(footprint)  # Full coverage isolates cow objectives.
                clock.publish(stamp)
                executor.spin_once(timeout_sec=.02)
            assert process.poll() is None, log.read_text()
            counts = re.findall(r'objectives=0/(\d+)', log.read_text()[offset:])
            assert counts and int(counts[-1]) == expected, log.read_text()

        # A at (4,0) wants to push from (8,0). B's own pushing goal stays free.
        check([(4, 0)], 1)
        check([(4, 0), (8, 0)], 1)       # Desired goal at B's center.
        check([(10, 0), (4, 0)], 1)      # Exactly on B's radius; reverse input order.
        check([(4, 0), (10.6, 0)], 2)    # B moves clear: A's goal returns.
        check([(4, 0)], 1)               # B disappears: A's goal stays available.
        # A cow between normal (3 m) and completion (5 m) still has a pushing
        # goal above. A new arrival within 3 m activates the 5 m no-push zone.
        check([(2.9, 0), (4.5, 0)], 0)
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.remove_node(driver)
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
