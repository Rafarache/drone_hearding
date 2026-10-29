import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class CowConfig:
    source_path: str
    drone_sensing_radius_m: float
    cow_max_repulsion: float
    cow_max_attraction: float
    cow_equilibrium_distance_m: float
    cow_cutoff_distance_m: float
    cow_repulsion_shape: float
    cow_attraction_rise: float
    cow_attraction_fall: float
    linear_velocity_mps: float
    full_speed_force: float
    force_stop_threshold: float
    max_angular_velocity_radps: float
    heading_proportional_gain: float
    heading_derivative_gain: float
    heading_tolerance_rad: float
    control_period_seconds: float


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


def load_cow_config(source_path=None):
    source_path = source_path or os.path.join(
        get_package_share_directory('cow_pkg'),
        'config',
        'cow_config.xml',
    )
    try:
        root = ET.parse(source_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(
            f'Unable to load cow configuration {source_path}: {exc}'
        ) from exc

    interaction = root.find('interaction')
    motion = root.find('motion')
    timing = root.find('timing')
    if interaction is None or motion is None or timing is None:
        raise ValueError(
            f'{source_path} requires interaction, motion, and timing sections'
        )

    config = CowConfig(
        source_path=source_path,
        drone_sensing_radius_m=_required_float(
            interaction, 'drone_sensing_radius_m', source_path
        ),
        cow_max_repulsion=_required_float(
            interaction, 'cow_max_repulsion', source_path
        ),
        cow_max_attraction=_required_float(
            interaction, 'cow_max_attraction', source_path
        ),
        cow_equilibrium_distance_m=_required_float(
            interaction, 'cow_equilibrium_distance_m', source_path
        ),
        cow_cutoff_distance_m=_required_float(
            interaction, 'cow_cutoff_distance_m', source_path
        ),
        cow_repulsion_shape=_required_float(
            interaction, 'cow_repulsion_shape', source_path
        ),
        cow_attraction_rise=_required_float(
            interaction, 'cow_attraction_rise', source_path
        ),
        cow_attraction_fall=_required_float(
            interaction, 'cow_attraction_fall', source_path
        ),
        linear_velocity_mps=_required_float(
            motion, 'linear_velocity_mps', source_path
        ),
        full_speed_force=_required_float(
            motion, 'full_speed_force', source_path
        ),
        force_stop_threshold=_required_float(
            motion, 'force_stop_threshold', source_path
        ),
        max_angular_velocity_radps=_required_float(
            motion, 'max_angular_velocity_radps', source_path
        ),
        heading_proportional_gain=_required_float(
            motion, 'heading_proportional_gain', source_path
        ),
        heading_derivative_gain=_required_float(
            motion, 'heading_derivative_gain', source_path
        ),
        heading_tolerance_rad=_required_float(
            motion, 'heading_tolerance_rad', source_path
        ),
        control_period_seconds=_required_float(
            timing, 'control_period_seconds', source_path
        ),
    )

    positive = {
        'drone_sensing_radius_m': config.drone_sensing_radius_m,
        'cow_equilibrium_distance_m': config.cow_equilibrium_distance_m,
        'cow_cutoff_distance_m': config.cow_cutoff_distance_m,
        'cow_repulsion_shape': config.cow_repulsion_shape,
        'cow_attraction_rise': config.cow_attraction_rise,
        'cow_attraction_fall': config.cow_attraction_fall,
        'linear_velocity_mps': config.linear_velocity_mps,
        'full_speed_force': config.full_speed_force,
        'force_stop_threshold': config.force_stop_threshold,
        'max_angular_velocity_radps': config.max_angular_velocity_radps,
        'heading_proportional_gain': config.heading_proportional_gain,
        'control_period_seconds': config.control_period_seconds,
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be positive in {source_path}')

    for name in ('cow_max_repulsion', 'cow_max_attraction'):
        if getattr(config, name) < 0.0:
            raise ValueError(f'{name} must be nonnegative in {source_path}')
    if config.heading_derivative_gain < 0.0:
        raise ValueError(f'heading_derivative_gain must be nonnegative in {source_path}')
    if not 0.0 <= config.heading_tolerance_rad < math.pi / 2:
        raise ValueError(f'heading_tolerance_rad must be in [0, pi/2) in {source_path}')
    if config.cow_cutoff_distance_m <= config.cow_equilibrium_distance_m:
        raise ValueError(
            'cow_cutoff_distance_m must exceed cow_equilibrium_distance_m '
            f'in {source_path}'
        )

    return config
