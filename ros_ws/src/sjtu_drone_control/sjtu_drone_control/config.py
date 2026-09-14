import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class DroneControlConfig:
    source_path: str
    max_linear_velocity_mps: float
    max_linear_acceleration_mps2: float
    max_angular_velocity_radps: float
    max_angular_acceleration_radps2: float
    position_tolerance_m: float
    linear_slowdown_distance_m: float
    focus_distance_tolerance_m: float
    focus_angle_tolerance_rad: float
    control_period_seconds: float
    max_control_dt_seconds: float
    takeoff_delay_seconds: float
    subscriber_wait_timeout_seconds: float
    subscriber_poll_period_seconds: float


def _required_float(element, attribute, source_path):
    value = element.get(attribute)
    if value is None:
        raise ValueError(
            f'Missing attribute {attribute!r} in {source_path}'
        )
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(
            f'Attribute {attribute!r} must be numeric in {source_path}'
        ) from exc
    if not math.isfinite(parsed):
        raise ValueError(
            f'Attribute {attribute!r} must be finite in {source_path}'
        )
    return parsed


def load_drone_control_config():
    source_path = os.path.join(
        get_package_share_directory('sjtu_drone_control'),
        'config',
        'drone_control_config.xml',
    )
    try:
        root = ET.parse(source_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(
            f'Unable to load drone control configuration {source_path}: {exc}'
        ) from exc

    motion = root.find('motion')
    focus = root.find('focus')
    timing = root.find('timing')
    if motion is None or focus is None or timing is None:
        raise ValueError(
            f'{source_path} requires motion, focus, and timing sections'
        )

    config = DroneControlConfig(
        source_path=source_path,
        max_linear_velocity_mps=_required_float(
            motion, 'max_linear_velocity_mps', source_path
        ),
        max_linear_acceleration_mps2=_required_float(
            motion, 'max_linear_acceleration_mps2', source_path
        ),
        max_angular_velocity_radps=_required_float(
            motion, 'max_angular_velocity_radps', source_path
        ),
        max_angular_acceleration_radps2=_required_float(
            motion, 'max_angular_acceleration_radps2', source_path
        ),
        position_tolerance_m=_required_float(
            motion, 'position_tolerance_m', source_path
        ),
        linear_slowdown_distance_m=_required_float(
            motion, 'linear_slowdown_distance_m', source_path
        ),
        focus_distance_tolerance_m=_required_float(
            focus, 'distance_tolerance_m', source_path
        ),
        focus_angle_tolerance_rad=_required_float(
            focus, 'angle_tolerance_rad', source_path
        ),
        control_period_seconds=_required_float(
            timing, 'control_period_seconds', source_path
        ),
        max_control_dt_seconds=_required_float(
            timing, 'max_control_dt_seconds', source_path
        ),
        takeoff_delay_seconds=_required_float(
            timing, 'takeoff_delay_seconds', source_path
        ),
        subscriber_wait_timeout_seconds=_required_float(
            timing, 'subscriber_wait_timeout_seconds', source_path
        ),
        subscriber_poll_period_seconds=_required_float(
            timing, 'subscriber_poll_period_seconds', source_path
        ),
    )

    positive = {
        'max_linear_velocity_mps': config.max_linear_velocity_mps,
        'max_linear_acceleration_mps2': config.max_linear_acceleration_mps2,
        'max_angular_velocity_radps': config.max_angular_velocity_radps,
        'max_angular_acceleration_radps2':
            config.max_angular_acceleration_radps2,
        'linear_slowdown_distance_m': config.linear_slowdown_distance_m,
        'focus_distance_tolerance_m': config.focus_distance_tolerance_m,
        'focus_angle_tolerance_rad': config.focus_angle_tolerance_rad,
        'control_period_seconds': config.control_period_seconds,
        'max_control_dt_seconds': config.max_control_dt_seconds,
        'subscriber_poll_period_seconds':
            config.subscriber_poll_period_seconds,
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be positive in {source_path}')

    nonnegative = {
        'position_tolerance_m': config.position_tolerance_m,
        'takeoff_delay_seconds': config.takeoff_delay_seconds,
        'subscriber_wait_timeout_seconds':
            config.subscriber_wait_timeout_seconds,
    }
    for name, value in nonnegative.items():
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative in {source_path}')

    if config.max_control_dt_seconds < config.control_period_seconds:
        raise ValueError(
            'max_control_dt_seconds must be at least control_period_seconds '
            f'in {source_path}'
        )

    return config
