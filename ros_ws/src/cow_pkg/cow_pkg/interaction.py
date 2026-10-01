"""Signed cow interaction: negative repels, positive attracts."""

import math


def interaction_force(
    distance,
    *,
    max_repulsion,
    max_attraction,
    equilibrium_distance,
    cutoff_distance,
    repulsion_shape,
    attraction_rise,
    attraction_fall,
):
    """Evaluate the cow interaction curve at a nonnegative distance."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError('distance must be finite and nonnegative')
    if distance < equilibrium_distance:
        return -max_repulsion * (
            1.0 - distance / equilibrium_distance
        ) ** repulsion_shape
    if distance >= cutoff_distance:
        return 0.0

    x = (distance - equilibrium_distance) / (
        cutoff_distance - equilibrium_distance
    )
    m = attraction_rise / (attraction_rise + attraction_fall)
    return (
        max_attraction
        * (x / m) ** attraction_rise
        * ((1.0 - x) / (1.0 - m)) ** attraction_fall
    )


def cow_interaction_vector(delta_x, delta_y, config):
    """Return force toward another cow, given its position minus our own."""
    distance = math.hypot(delta_x, delta_y)
    # Coincident centers have no defined direction; avoid division by zero.
    if distance == 0.0:
        return 0.0, 0.0
    magnitude = interaction_force(
        distance,
        max_repulsion=config.cow_max_repulsion,
        max_attraction=config.cow_max_attraction,
        equilibrium_distance=config.cow_equilibrium_distance_m,
        cutoff_distance=config.cow_cutoff_distance_m,
        repulsion_shape=config.cow_repulsion_shape,
        attraction_rise=config.cow_attraction_rise,
        attraction_fall=config.cow_attraction_fall,
    )
    return magnitude * delta_x / distance, magnitude * delta_y / distance
