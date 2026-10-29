"""Existing empirical camera projection, with full box bearings for occlusion."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ProjectedDetection:
    position_valid: bool
    x: float
    y: float
    bearing_min: float
    bearing_max: float
    distance: float


def pixel_bearing(config, pixel_x):
    offset = pixel_x - config.image_center_x_px
    image_angle = math.log(
        (abs(offset) + config.bearing_center_offset_px)
        / config.bearing_log_denominator) / config.bearing_log_scale
    return -math.copysign(1.0, offset) * math.radians(image_angle)


def project_box(config, x1, x2, bottom, drone_state):
    """Invalid depth still supplies a bearing shadow, never empty evidence."""
    bearings = [pixel_bearing(config, x1), pixel_bearing(config, x2)]
    # The old calibration has a small discontinuity at the optical center.
    if x1 <= config.image_center_x_px <= x2:
        bearings.extend([0.0, pixel_bearing(config, config.image_center_x_px - 1e-8),
                         pixel_bearing(config, config.image_center_x_px + 1e-8)])
    center_bearing = pixel_bearing(config, (x1 + x2) / 2.0)
    distance = math.nan
    if bottom >= config.minimum_box_bottom_px and bottom > config.distance_bottom_offset_px:
        forward = -math.log((bottom - config.distance_bottom_offset_px)
                            / config.distance_log_denominator) / config.distance_log_scale
        distance = abs(forward / math.cos(center_bearing))
    valid = math.isfinite(distance) and 0.0 < distance <= config.max_detection_distance_m
    x, y, yaw = drone_state
    return ProjectedDetection(
        valid,
        float(x + math.cos(yaw + center_bearing) * distance) if valid else 0.0,
        float(y + math.sin(yaw + center_bearing) * distance) if valid else 0.0,
        min(bearings), max(bearings), distance)
