"""Convert the resultant force to bounded cow velocity commands."""

import math


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def force_to_velocity(force_x, force_y, yaw, last_yaw, config):
    magnitude = math.hypot(force_x, force_y)
    if magnitude <= config.force_stop_threshold:
        return 0.0, 0.0

    heading_error = normalize_angle(math.atan2(force_y, force_x) - yaw)
    # Do not drive away from the resultant while turning around. Project
    # forward speed onto the desired direction and slow down near equilibrium.
    alignment = (
        math.cos(heading_error) if abs(heading_error) < math.pi / 2 else 0.0
    )
    linear = (
        config.linear_velocity_mps
        * min(magnitude / config.full_speed_force, 1.0)
        * alignment
    )
    # Wrap the heading change itself to avoid a spike across +/- pi.
    angular = (
        config.heading_proportional_gain * heading_error
        + config.heading_derivative_gain * normalize_angle(last_yaw - yaw)
    )
    angular = max(
        -config.max_angular_velocity_radps,
        min(config.max_angular_velocity_radps, angular),
    )
    return linear, angular
