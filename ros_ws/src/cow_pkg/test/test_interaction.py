import math
from types import SimpleNamespace
import unittest

from cow_pkg.interaction import cow_interaction_vector, interaction_force


PARAMETERS = dict(
    max_repulsion=2.05,
    max_attraction=2.3,
    equilibrium_distance=4.0,
    cutoff_distance=18.5,
    repulsion_shape=2.3,
    attraction_rise=1.2,
    attraction_fall=6.2,
)


def force_config():
    return SimpleNamespace(
        cow_max_repulsion=PARAMETERS['max_repulsion'],
        cow_max_attraction=PARAMETERS['max_attraction'],
        cow_equilibrium_distance_m=PARAMETERS['equilibrium_distance'],
        cow_cutoff_distance_m=PARAMETERS['cutoff_distance'],
        cow_repulsion_shape=PARAMETERS['repulsion_shape'],
        cow_attraction_rise=PARAMETERS['attraction_rise'],
        cow_attraction_fall=PARAMETERS['attraction_fall'],
    )


class InteractionTests(unittest.TestCase):
    def test_maximum_repulsion_and_repulsion_shape(self):
        self.assertEqual(interaction_force(0.0, **PARAMETERS), -2.05)
        self.assertAlmostEqual(
            interaction_force(2.0, **PARAMETERS), -2.05 * 0.5 ** 2.3
        )

    def test_equilibrium_and_cutoff_are_zero(self):
        for distance in (4.0, 18.5, 25.0):
            self.assertEqual(interaction_force(distance, **PARAMETERS), 0.0)

    def test_attraction_reaches_configured_maximum(self):
        peak = 4.0 + (18.5 - 4.0) * 1.2 / (1.2 + 6.2)
        self.assertAlmostEqual(interaction_force(peak, **PARAMETERS), 2.3)
        for distance in (4.01, peak - 0.1, peak + 0.1, 18.49):
            force = interaction_force(distance, **PARAMETERS)
            self.assertGreater(force, 0.0)
            self.assertLess(force, 2.3)

    def test_curve_is_continuous_at_equilibrium_and_cutoff(self):
        for boundary in (4.0, 18.5):
            for offset in (-1e-7, 1e-7):
                self.assertAlmostEqual(
                    interaction_force(boundary + offset, **PARAMETERS),
                    0.0, places=6,
                )

    def test_vector_repels_near_and_attracts_far(self):
        config = force_config()
        near = cow_interaction_vector(2.0, 0.0, config)
        far = cow_interaction_vector(6.0, 0.0, config)
        self.assertLess(near[0], 0.0)
        self.assertGreater(far[0], 0.0)
        self.assertEqual(near[1], 0.0)
        self.assertEqual(far[1], 0.0)

    def test_vector_is_equal_and_opposite_for_pair(self):
        config = force_config()
        for dx, dy in ((1.0, 2.0), (5.0, 3.0)):
            first = cow_interaction_vector(dx, dy, config)
            second = cow_interaction_vector(-dx, -dy, config)
            self.assertEqual(first, (-second[0], -second[1]))
            self.assertAlmostEqual(
                math.hypot(*first),
                abs(interaction_force(math.hypot(dx, dy), **PARAMETERS)),
            )

    def test_coincident_cows_and_beyond_cutoff_are_safe(self):
        config = force_config()
        for dx, dy in ((0.0, 0.0), (20.0, 0.0)):
            self.assertEqual(cow_interaction_vector(dx, dy, config), (0.0, 0.0))

    def test_invalid_distance_is_rejected(self):
        for distance in (-1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                interaction_force(distance, **PARAMETERS)
