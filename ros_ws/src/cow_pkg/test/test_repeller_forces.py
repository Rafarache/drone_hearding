import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

from cow_pkg.repeller_node import PeopleRepeller
from test_interaction import force_config


def pose(x, y):
    result = Pose()
    result.position.x = float(x)
    result.position.y = float(y)
    result.orientation.w = 1.0
    return result


class RepellerForceTests(unittest.TestCase):
    def test_model_state_rate_is_independent_of_message_frequency(self):
        state = SimpleNamespace(publishers_dict={'cow0': Mock()}, cows_dict=None,
                                cow_yaw_rates={})
        sample = ModelStates(name=['cow0'], pose=[pose(0, 0)],
                             twist=[Twist()])
        sample.twist[0].angular.z = 0.3
        for _ in range(100):
            PeopleRepeller.models_callback(state, sample)
            self.assertEqual(state.cow_yaw_rates['cow0'], 0.3)
        # A snapshot without Twist data must not reuse a stale angular rate.
        sample.twist = []
        PeopleRepeller.models_callback(state, sample)
        self.assertEqual(state.cow_yaw_rates['cow0'], 0.0)

    def test_missing_cow_clears_previous_movement_command(self):
        publisher = Mock()
        state = SimpleNamespace(stopped=False, cows_dict={}, publishers_dict={'cow0': publisher})
        PeopleRepeller.move_cows(state)
        self.assertEqual(publisher.publish.call_args.args[0], Twist())

    def command(self, cows, drones, *, yaw_rates=None):
        config = force_config()
        config.drone_sensing_radius_m = 6.0
        config.linear_velocity_mps = 0.5
        config.full_speed_force = 1.0
        config.force_stop_threshold = 1e-6
        config.max_angular_velocity_radps = 1.0
        config.heading_proportional_gain = 0.2
        config.heading_derivative_gain = 5.0
        config.heading_tolerance_rad = 0.05
        publisher = Mock()
        state = SimpleNamespace(
            config=config,
            stopped=False,
            cows_dict={'cow0': pose(0, 0), **cows},
            cow_yaw_rates=yaw_rates or {},
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
        self.assertAlmostEqual(command.angular.z, -math.pi / 2 * 0.2 * 0.25)

    def test_drone_force_keeps_inverse_square_magnitude(self):
        for distance in (2.0, 4.0):
            command = self.command({}, {'drone0': pose(-distance, 0)})
            self.assertAlmostEqual(command.linear.x, 0.5 / distance ** 2)
            self.assertEqual(command.angular.z, 0.0)

    def test_measured_angular_velocity_damps_the_command(self):
        command = self.command({}, {'drone0': pose(-4, 0)}, yaw_rates={'cow0': 0.2})
        self.assertAlmostEqual(command.angular.z, -(1.0 / 16) * 5.0 * 0.2)

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
