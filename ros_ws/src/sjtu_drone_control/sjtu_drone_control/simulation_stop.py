"""Permanent stop subscription shared by the supported drone controllers."""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class SimulationStop:
    def __init__(self, node, on_stop):
        self.stopped = False
        self.on_stop = on_stop
        self.subscription = node.create_subscription(
            Empty, '/simulation/stop', self._receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _receive(self, _msg):
        self.stopped = True
        self.on_stop()
