import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import Pose

from cow_pkg.repeller_node import PeopleRepeller
from test_interaction import force_config


def pose(x, y):
    result = Pose()
    result.position.x = float(x)
    result.position.y = float(y)
    result.orientation.w = 1.0
    return result


class RepellerForceTests(unittest.TestCase):
    def command(self, cows, drones):
        config = force_config()
        config.drone_sensing_radius_m = 6.0
        config.linear_velocity_mps = 0.5
        config.full_speed_force = 1.0
        config.force_stop_threshold = 1e-6
        config.max_angular_velocity_radps = 1.0
        config.heading_proportional_gain = 0.2
        config.heading_derivative_gain = 5.0
        publisher = Mock()
        state = SimpleNamespace(
            config=config,
            cows_dict={'cow0': pose(0, 0), **cows},
            last_cows_dict=None,
            publishers_dict={'cow0': publisher},
            drone_pos_dir=drones,
            quaternion_to_yaw=PeopleRepeller.quaternion_to_yaw,
            normalize_rad_neg_pi_to_pi=(
                PeopleRepeller.normalize_rad_neg_pi_to_pi
            ),
        )
        PeopleRepeller.move_cows(state)
        return publisher.publish.call_args.args[0]

    def test_drone_to_side_turns_without_forward_drift(self):
        command = self.command({}, {'drone0': pose(0, 2)})
        self.assertEqual(command.linear.x, 0.0)
        self.assertAlmostEqual(command.angular.z, -math.pi / 2 * 0.2)

    def test_drone_force_keeps_inverse_square_magnitude(self):
        for distance in (2.0, 4.0):
            command = self.command({}, {'drone0': pose(-distance, 0)})
            self.assertAlmostEqual(command.linear.x, 0.5 / distance ** 2)
            self.assertEqual(command.angular.z, 0.0)

    def test_cow_and_drone_forces_are_summed_before_steering(self):
        peak = 4.0 + (18.5 - 4.0) * 1.2 / (1.2 + 6.2)
        command = self.command(
            {'cow1': pose(peak, 0)}, {'drone0': pose(0, 2)}
        )
        self.assertAlmostEqual(
            command.linear.x, 0.5 * math.cos(math.atan2(-0.25, 2.3))
        )
        self.assertAlmostEqual(command.angular.z, math.atan2(-0.25, 2.3) * 0.2)

    def test_balanced_cow_forces_stop_cow(self):
        command = self.command(
            {'cow1': pose(6, 0), 'cow2': pose(-6, 0)}, {}
        )
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.0)
