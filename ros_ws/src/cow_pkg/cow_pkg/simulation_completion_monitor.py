"""Latch mission completion from Gazebo truth, keeping the simulation open."""

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty

from cow_pkg.completion import all_cows_at_goal


class SimulationCompletionMonitor(Node):
    def __init__(self):
        super().__init__('simulation_completion_monitor')
        prefix = self.declare_parameter('cow_name_prefix', 'cow').value
        count = self.declare_parameter('number_of_cows', 0).value
        self.goal = (self.declare_parameter('goal_x', -10.0).value,
                     self.declare_parameter('goal_y', -10.0).value)
        self.radius = self.declare_parameter('final_goal_reached_radius', 2.0).value
        if count < 1 or not prefix or not all(
                math.isfinite(v) for v in (*self.goal, self.radius)) or self.radius <= 0:
            raise ValueError('Invalid completion parameters')
        self.expected = [prefix + str(i) for i in range(count)]
        self.completed = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Empty, '/simulation/stop', qos)
        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.check, 10)

    def check(self, msg):
        if not self.completed and all_cows_at_goal(
                msg.name, msg.pose, self.expected, self.goal, self.radius):
            self.completed = True
            self.publisher.publish(Empty())
            self.get_logger().info('All configured cows reached the goal; movement stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = SimulationCompletionMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
