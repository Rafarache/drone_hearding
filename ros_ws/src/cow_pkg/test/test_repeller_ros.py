"""Exercise cow steering through ROS topics in an isolated ROS domain."""

import math
import time

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty

from cow_pkg.repeller_node import PeopleRepeller


def pose(x, y):
    result = Pose()
    result.position.x, result.position.y = float(x), float(y)
    result.orientation.w = 1.0
    return result


def test_angular_feedback_weak_forces_and_completion_stop():
    rclpy.init(args=['--ros-args', '-p', 'number_of_cows:=2',
                     '-p', 'number_of_drones:=1', '-p', 'cow_name_prefix:=turn_cow',
                     '-p', 'drone_namespace_prefix:=/turn_drone',
                     '-p', 'drone_approach_radius:=2.0', '-p', 'push_point_margin:=2.0'])
    driver, repeller = Node('cow_turn_test_driver'), PeopleRepeller()
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(repeller)
    commands = []
    subscription = driver.create_subscription(Twist, '/turn_cow0/cmd_vel', commands.append, 10)
    models = driver.create_publisher(ModelStates, '/gazebo/model_states', 10)
    drones = driver.create_publisher(Pose, '/turn_drone0/gt_pose', 10)
    stop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    stopper = driver.create_publisher(Empty, '/simulation/stop', stop_qos)

    def pump(other_x, drone_x, rate, duration=0.8):
        deadline = time.monotonic() + duration
        tick = 0
        while time.monotonic() < deadline:
            cow_twist = Twist()
            cow_twist.angular.z = float(rate)
            state = ModelStates(name=['turn_cow0', 'turn_cow1'],
                                pose=[pose(0, 0), pose(other_x, 0)],
                                twist=[cow_twist, Twist()])
            # Gazebo array order is not an identity and can change on spawn.
            if tick % 2:
                state.name = list(reversed(state.name))
                state.pose = list(reversed(state.pose))
                state.twist = list(reversed(state.twist))
            models.publish(state)
            drones.publish(pose(drone_x, 0))
            executor.spin_once(timeout_sec=0.01)
            tick += 1

    try:
        pump(30, -4, .4, 1.2)
        assert commands
        # Force=1/16, scale=(1/16)/0.1; rate damping uses the measured .4 rad/s.
        expected = -(1 / 16 / repeller.config.full_speed_force) * (
            repeller.config.heading_derivative_gain * .4)
        assert abs(commands[-1].angular.z - expected) < 1e-9
        assert commands[-1].linear.x > 0
        # A near-equilibrium cow force points behind this cow but is too small
        # to justify continuous rotation. Move the drone out of sensing range.
        commands.clear()
        separation = repeller.config.cow_equilibrium_distance_m * (
            1 - (.0009 / repeller.config.cow_max_repulsion) **
            (1 / repeller.config.cow_repulsion_shape))
        pump(separation, -100, 0)
        assert commands and commands[-1] == Twist()
        assert math.isfinite(repeller.cow_yaw_rates['turn_cow0'])
        # Stop remains latched even if a strong movement force is restored.
        stopper.publish(Empty())
        pump(30, -4, 0)
        assert repeller.stopped and commands[-1] == Twist()
        assert subscription  # Keep the subscriber alive throughout the test.
    finally:
        executor.remove_node(driver)
        executor.remove_node(repeller)
        driver.destroy_node()
        repeller.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
