"""Convert the resultant force to bounded cow velocity commands."""

import math


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def force_to_velocity(force_x, force_y, yaw, config, *, yaw_rate=0.0):
    """Steer using measured angular velocity in rad/s, independent of pose cadence."""
    if not all(math.isfinite(value) for value in (force_x, force_y, yaw, yaw_rate)):
        return 0.0, 0.0
    magnitude = math.hypot(force_x, force_y)
    if magnitude <= config.force_stop_threshold:
        return 0.0, 0.0

    force_scale = min(magnitude / config.full_speed_force, 1.0)
    heading_error = normalize_angle(math.atan2(force_y, force_x) - yaw)
    # Do not drive away from the resultant while turning around. Project
    # forward speed onto the desired direction and slow down near equilibrium.
    alignment = (
        math.cos(heading_error) if abs(heading_error) < math.pi / 2 else 0.0
    )
    linear = (
        config.linear_velocity_mps
        * force_scale
        * alignment
    )
    # Weak/cancelling forces must not request full-strength rotation while
    # barely translating. Damping uses Gazebo's angular velocity rather than
    # an unscaled difference between arbitrarily spaced model-state messages.
    steering_error = heading_error if abs(heading_error) > config.heading_tolerance_rad else 0.0
    angular = force_scale * (
        config.heading_proportional_gain * steering_error
        - config.heading_derivative_gain * yaw_rate
    )
    angular = max(
        -config.max_angular_velocity_radps,
        min(config.max_angular_velocity_radps, angular),
    )
    return linear, angular
