import math
from types import SimpleNamespace
import unittest

from cow_pkg.interaction import cow_interaction_vector
from cow_pkg.motion import force_to_velocity


def motion_config():
    return SimpleNamespace(
        linear_velocity_mps=0.5,
        full_speed_force=1.0,
        force_stop_threshold=1e-6,
        max_angular_velocity_radps=1.0,
        heading_proportional_gain=0.2,
        heading_derivative_gain=5.0,
    )


class CowMotionTests(unittest.TestCase):
    def test_small_resultants_stop_both_translation_and_rotation(self):
        config = motion_config()
        for fx, fy in ((0.0, 0.0), (0.0, 1e-7), (1e-6, 0.0)):
            self.assertEqual(
                force_to_velocity(fx, fy, 1.0, 0.5, config), (0.0, 0.0)
            )

    def test_forward_speed_decreases_with_force_and_is_capped(self):
        config = motion_config()
        for force in (0.001, 0.1, 0.5, 1.0, 100.0):
            linear, angular = force_to_velocity(force, 0.0, 0.0, 0.0, config)
            self.assertAlmostEqual(linear, 0.5 * min(force, 1.0))
            self.assertEqual(angular, 0.0)

    def test_facing_away_turns_in_place(self):
        linear, angular = force_to_velocity(-0.1, 0.0, 0.0, 0.0, motion_config())
        self.assertEqual(linear, 0.0)
        self.assertGreater(abs(angular), 0.0)

    def test_translation_never_opposes_resultant(self):
        for degrees in range(-180, 181, 10):
            yaw = math.radians(degrees)
            linear, _ = force_to_velocity(0.1, 0.0, yaw, yaw, motion_config())
            self.assertGreaterEqual(linear * math.cos(yaw), 0.0)

    def test_wrapping_heading_does_not_create_turn_spike(self):
        config = motion_config()
        yaw = -math.pi + 0.001
        linear, angular = force_to_velocity(
            math.cos(yaw), math.sin(yaw), yaw, math.pi - 0.001, config
        )
        self.assertAlmostEqual(linear, 0.5)
        self.assertAlmostEqual(angular, -0.01)

    def test_angular_speed_is_bounded(self):
        _, angular = force_to_velocity(0.1, 0.0, 1.0, -1.0, motion_config())
        self.assertEqual(angular, -1.0)

    def test_pair_at_5_hz_approaches_equilibrium_without_center_drift(self):
        config = motion_config()
        config.cow_max_repulsion = 0.1
        config.cow_max_attraction = 0.1
        config.cow_equilibrium_distance_m = 4.0
        config.cow_cutoff_distance_m = 18.5
        config.cow_repulsion_shape = 8.0
        config.cow_attraction_rise = 6.0
        config.cow_attraction_fall = 6.0
        for initial_distance in (2.0, 6.0):
            left, right = -initial_distance / 2.0, initial_distance / 2.0
            left_yaw = 0.0 if initial_distance > 4.0 else math.pi
            right_yaw = math.pi if initial_distance > 4.0 else 0.0
            previous_error = abs(initial_distance - 4.0)
            for _ in range(1000):
                fx, fy = cow_interaction_vector(right - left, 0.0, config)
                left_speed, _ = force_to_velocity(
                    fx, fy, left_yaw, left_yaw, config
                )
                right_speed, _ = force_to_velocity(
                    -fx, -fy, right_yaw, right_yaw, config
                )
                left += 0.2 * left_speed * math.cos(left_yaw)
                right += 0.2 * right_speed * math.cos(right_yaw)
                error = abs(right - left - 4.0)
                self.assertLessEqual(error, previous_error)
                self.assertAlmostEqual(left + right, 0.0)
                previous_error = error
            self.assertLess(previous_error, abs(initial_distance - 4.0))
