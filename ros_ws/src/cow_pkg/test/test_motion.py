import math
from types import SimpleNamespace
import unittest

from cow_pkg.interaction import cow_interaction_vector
from cow_pkg.motion import force_to_velocity, normalize_angle
from cow_pkg.config import load_cow_config


def motion_config():
    return SimpleNamespace(
        linear_velocity_mps=0.5,
        full_speed_force=1.0,
        force_stop_threshold=1e-6,
        max_angular_velocity_radps=1.0,
        heading_proportional_gain=0.2,
        heading_derivative_gain=5.0,
        heading_tolerance_rad=0.05,
    )


class CowMotionTests(unittest.TestCase):
    def test_weak_force_reduces_turning_as_well_as_translation(self):
        config = motion_config()
        strong = force_to_velocity(0.0, 1.0, 0.0, config)
        weak = force_to_velocity(0.0, 0.001, 0.0, config)
        self.assertAlmostEqual(weak[1], strong[1] * 0.001)

    def test_heading_deadband_and_rate_damping(self):
        config = motion_config()
        self.assertEqual(force_to_velocity(1.0, 0.01, 0.0, config)[1], 0.0)
        self.assertLess(force_to_velocity(1.0, 0.0, 0.0, config, yaw_rate=0.1)[1], 0.0)
        self.assertGreater(force_to_velocity(1.0, 0.0, 0.0, config, yaw_rate=-0.1)[1], 0.0)

    def test_nonfinite_inputs_hold_instead_of_leaving_a_turn_command(self):
        config = motion_config()
        for invalid in (float('nan'), float('inf'), -float('inf')):
            for values in ((invalid, 0.0, 0.0, 0.0), (1.0, invalid, 0.0, 0.0),
                           (1.0, 0.0, invalid, 0.0), (1.0, 0.0, 0.0, invalid)):
                fx, fy, yaw, rate = values
                self.assertEqual(force_to_velocity(fx, fy, yaw, config, yaw_rate=rate), (0.0, 0.0))

    def test_small_resultants_stop_both_translation_and_rotation(self):
        config = motion_config()
        for fx, fy in ((0.0, 0.0), (0.0, 1e-7), (1e-6, 0.0)):
            self.assertEqual(
                force_to_velocity(fx, fy, 1.0, config, yaw_rate=0.5), (0.0, 0.0)
            )

    def test_forward_speed_decreases_with_force_and_is_capped(self):
        config = motion_config()
        for force in (0.001, 0.1, 0.5, 1.0, 100.0):
            linear, angular = force_to_velocity(force, 0.0, 0.0, config)
            self.assertAlmostEqual(linear, 0.5 * min(force, 1.0))
            self.assertEqual(angular, 0.0)

    def test_facing_away_turns_in_place(self):
        linear, angular = force_to_velocity(-0.1, 0.0, 0.0, motion_config())
        self.assertEqual(linear, 0.0)
        self.assertGreater(abs(angular), 0.0)

    def test_translation_never_opposes_resultant(self):
        for degrees in range(-180, 181, 10):
            yaw = math.radians(degrees)
            linear, _ = force_to_velocity(0.1, 0.0, yaw, motion_config())
            self.assertGreaterEqual(linear * math.cos(yaw), 0.0)

    def test_wrapping_heading_does_not_create_turn_spike(self):
        config = motion_config()
        yaw = -math.pi + 0.001
        linear, angular = force_to_velocity(
            math.cos(yaw), math.sin(yaw), yaw, config, yaw_rate=0.01
        )
        self.assertAlmostEqual(linear, 0.5)
        self.assertAlmostEqual(angular, -0.05)

    def test_angular_speed_is_bounded(self):
        _, angular = force_to_velocity(100.0, 0.0, 1.0, motion_config(), yaw_rate=1.0)
        self.assertEqual(angular, -1.0)

    def test_configured_heading_converges_instead_of_spinning(self):
        config = load_cow_config()
        for initial_yaw in (-math.pi + .001, math.pi - .001, -1.5, 1.5):
            yaw, rate, total_rotation = initial_yaw, 0.0, 0.0
            for _ in range(150):
                _, rate = force_to_velocity(1.0, 0.0, yaw, config, yaw_rate=rate)
                total_rotation += abs(rate) * config.control_period_seconds
                yaw = normalize_angle(yaw + rate * config.control_period_seconds)
            self.assertLessEqual(abs(yaw), config.heading_tolerance_rad)
            self.assertLess(abs(rate), 1e-6)
            self.assertLessEqual(total_rotation, abs(initial_yaw) + .05)

    def test_configured_near_equilibrium_noise_does_not_rotate(self):
        config = load_cow_config()
        for i in range(200):
            heading = i * .71
            fx, fy = .0009 * math.cos(heading), .0009 * math.sin(heading)
            self.assertEqual(force_to_velocity(fx, fy, 0.0, config), (0.0, 0.0))
        _, angular = force_to_velocity(-.0011, 0.0, 0.0, config)
        self.assertLess(abs(angular), .04)

    def test_configured_pair_separates_before_body_contact_under_drone_pressure(self):
        config = load_cow_config()
        # 2.0 x 0.8 m cow bodies, initially approaching head-on. Each drone
        # stays 4 m behind its cow, applying the existing inverse-square force.
        left, right, left_velocity, right_velocity = -2.2, 2.2, 0.0, 0.0
        minimum_distance = right - left
        for _ in range(500):
            fx, fy = cow_interaction_vector(right-left, 0.0, config)
            force = fx + 1.0 / 4.0 ** 2
            left_speed, _ = force_to_velocity(force, fy, 0.0, config)
            right_speed, _ = force_to_velocity(-force, -fy, math.pi, config)
            # Include braking lag at 0.5 m/s^2 rather than stopping instantly.
            change = .5 * config.control_period_seconds
            left_velocity += max(-change, min(change, left_speed-left_velocity))
            right_velocity += max(-change, min(change, -right_speed-right_velocity))
            left += left_velocity * config.control_period_seconds
            right += right_velocity * config.control_period_seconds
            minimum_distance = min(minimum_distance, right-left)
        self.assertLess(minimum_distance, 4.4)  # The test actually closes the gap.
        self.assertGreater(minimum_distance, 2.5)
        # Formerly attractive at 4.5 m; this region now repels.
        self.assertLess(cow_interaction_vector(4.5, 0.0, config)[0], 0.0)

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
                    fx, fy, left_yaw, config
                )
                right_speed, _ = force_to_velocity(
                    -fx, -fy, right_yaw, config
                )
                left += 0.2 * left_speed * math.cos(left_yaw)
                right += 0.2 * right_speed * math.cos(right_yaw)
                error = abs(right - left - 4.0)
                self.assertLessEqual(error, previous_error)
                self.assertAlmostEqual(left + right, 0.0)
                previous_error = error
            self.assertLess(previous_error, abs(initial_distance - 4.0))
