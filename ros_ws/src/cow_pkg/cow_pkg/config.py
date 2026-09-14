import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class CowConfig:
    source_path: str
    drone_sensing_radius_m: float
    cow_repulsion_radius_m: float
    linear_velocity_mps: float
    heading_proportional_gain: float
    heading_derivative_gain: float
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


def load_cow_config():
    source_path = os.path.join(
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
        cow_repulsion_radius_m=_required_float(
            interaction, 'cow_repulsion_radius_m', source_path
        ),
        linear_velocity_mps=_required_float(
            motion, 'linear_velocity_mps', source_path
        ),
        heading_proportional_gain=_required_float(
            motion, 'heading_proportional_gain', source_path
        ),
        heading_derivative_gain=_required_float(
            motion, 'heading_derivative_gain', source_path
        ),
        control_period_seconds=_required_float(
            timing, 'control_period_seconds', source_path
        ),
    )

    positive = {
        'drone_sensing_radius_m': config.drone_sensing_radius_m,
        'cow_repulsion_radius_m': config.cow_repulsion_radius_m,
        'linear_velocity_mps': config.linear_velocity_mps,
        'control_period_seconds': config.control_period_seconds,
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be positive in {source_path}')

    return config
