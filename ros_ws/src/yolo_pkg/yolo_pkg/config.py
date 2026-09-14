import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class YoloConfig:
    source_path: str
    show_realtime_track_plot: bool
    show_annotated_camera: bool
    plot_history_ms: int
    plot_grid_m: float
    plot_min_x: float
    plot_max_x: float
    plot_min_y: float
    plot_max_y: float
    model_path: str
    tracked_labels: tuple
    confidence_threshold: float
    max_detection_distance_m: float
    horizontal_fov_rad: float
    pose_history_length: int
    max_pose_time_error_seconds: float
    multi_camera_merge_distance_m: float
    max_camera_batch_wait_seconds: float
    minimum_box_bottom_px: float
    image_center_x_px: float
    distance_log_scale: float
    distance_bottom_offset_px: float
    distance_log_denominator: float
    bearing_log_scale: float
    bearing_center_offset_px: float
    bearing_log_denominator: float
    tracker_default_dt_seconds: float
    tracker_max_lost_frames: int
    tracker_mahalanobis_gate: float
    tracker_max_position_distance_m: float
    tracker_min_confirmation_hits: int
    tracker_max_prediction_step_seconds: float
    tracker_acceleration_noise_std: float
    tracker_measurement_noise_std: float
    tracker_initial_velocity_std: float
    tracker_reacquisition_mahalanobis_gate: float
    tracker_reacquisition_distance_m: float
    tracker_diagnostic_log_period_seconds: float


def _required(element, attribute, source_path):
    value = element.get(attribute)
    if value is None:
        raise ValueError(
            f'Missing attribute {attribute!r} in {source_path}'
        )
    return value.strip()


def _required_float(element, attribute, source_path):
    raw = _required(element, attribute, source_path)
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(
            f'Attribute {attribute!r} must be numeric in {source_path}'
        ) from exc
    if not math.isfinite(value):
        raise ValueError(
            f'Attribute {attribute!r} must be finite in {source_path}'
        )
    return value


def _required_int(element, attribute, source_path):
    raw = _required(element, attribute, source_path)
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(
            f'Attribute {attribute!r} must be an integer in {source_path}'
        ) from exc


def _required_bool(element, attribute, source_path):
    raw = _required(element, attribute, source_path).lower()
    if raw not in {'true', 'false'}:
        raise ValueError(
            f'Attribute {attribute!r} must be true or false in {source_path}'
        )
    return raw == 'true'


def load_yolo_config():
    source_path = os.path.join(
        get_package_share_directory('yolo_pkg'),
        'config',
        'yolo_config.xml',
    )
    try:
        root = ET.parse(source_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(
            f'Unable to load YOLO configuration {source_path}: {exc}'
        ) from exc

    display = root.find('display')
    detector = root.find('detector')
    camera = root.find('camera')
    projection = root.find('projection')
    tracker = root.find('tracker')
    sections = (display, detector, camera, projection, tracker)
    if any(section is None for section in sections):
        raise ValueError(
            f'{source_path} requires display, detector, camera, projection, '
            'and tracker sections'
        )

    labels = tuple(
        label.strip()
        for label in _required(
            detector, 'tracked_labels', source_path
        ).split(',')
        if label.strip()
    )

    config = YoloConfig(
        source_path=source_path,
        show_realtime_track_plot=_required_bool(
            display, 'show_realtime_track_plot', source_path
        ),
        show_annotated_camera=_required_bool(
            display, 'show_annotated_camera', source_path
        ),
        plot_history_ms=_required_int(
            display, 'plot_history_ms', source_path
        ),
        plot_grid_m=_required_float(display, 'plot_grid_m', source_path),
        plot_min_x=_required_float(display, 'plot_min_x', source_path),
        plot_max_x=_required_float(display, 'plot_max_x', source_path),
        plot_min_y=_required_float(display, 'plot_min_y', source_path),
        plot_max_y=_required_float(display, 'plot_max_y', source_path),
        model_path=_required(detector, 'model_path', source_path),
        tracked_labels=labels,
        confidence_threshold=_required_float(
            detector, 'confidence_threshold', source_path
        ),
        max_detection_distance_m=_required_float(
            detector, 'max_detection_distance_m', source_path
        ),
        horizontal_fov_rad=_required_float(
            camera, 'horizontal_fov_rad', source_path
        ),
        pose_history_length=_required_int(
            camera, 'pose_history_length', source_path
        ),
        max_pose_time_error_seconds=_required_float(
            camera, 'max_pose_time_error_seconds', source_path
        ),
        multi_camera_merge_distance_m=_required_float(
            camera, 'multi_camera_merge_distance_m', source_path
        ),
        max_camera_batch_wait_seconds=_required_float(
            camera, 'max_camera_batch_wait_seconds', source_path
        ),
        minimum_box_bottom_px=_required_float(
            projection, 'minimum_box_bottom_px', source_path
        ),
        image_center_x_px=_required_float(
            projection, 'image_center_x_px', source_path
        ),
        distance_log_scale=_required_float(
            projection, 'distance_log_scale', source_path
        ),
        distance_bottom_offset_px=_required_float(
            projection, 'distance_bottom_offset_px', source_path
        ),
        distance_log_denominator=_required_float(
            projection, 'distance_log_denominator', source_path
        ),
        bearing_log_scale=_required_float(
            projection, 'bearing_log_scale', source_path
        ),
        bearing_center_offset_px=_required_float(
            projection, 'bearing_center_offset_px', source_path
        ),
        bearing_log_denominator=_required_float(
            projection, 'bearing_log_denominator', source_path
        ),
        tracker_default_dt_seconds=_required_float(
            tracker, 'default_dt_seconds', source_path
        ),
        tracker_max_lost_frames=_required_int(
            tracker, 'max_lost_frames', source_path
        ),
        tracker_mahalanobis_gate=_required_float(
            tracker, 'mahalanobis_gate', source_path
        ),
        tracker_max_position_distance_m=_required_float(
            tracker, 'max_position_distance_m', source_path
        ),
        tracker_min_confirmation_hits=_required_int(
            tracker, 'min_confirmation_hits', source_path
        ),
        tracker_max_prediction_step_seconds=_required_float(
            tracker, 'max_prediction_step_seconds', source_path
        ),
        tracker_acceleration_noise_std=_required_float(
            tracker, 'acceleration_noise_std', source_path
        ),
        tracker_measurement_noise_std=_required_float(
            tracker, 'measurement_noise_std', source_path
        ),
        tracker_initial_velocity_std=_required_float(
            tracker, 'initial_velocity_std', source_path
        ),
        tracker_reacquisition_mahalanobis_gate=_required_float(
            tracker, 'reacquisition_mahalanobis_gate', source_path
        ),
        tracker_reacquisition_distance_m=_required_float(
            tracker, 'reacquisition_distance_m', source_path
        ),
        tracker_diagnostic_log_period_seconds=_required_float(
            tracker, 'diagnostic_log_period_seconds', source_path
        ),
    )

    if not config.model_path:
        raise ValueError(f'model_path cannot be empty in {source_path}')
    if not config.tracked_labels:
        raise ValueError(f'tracked_labels cannot be empty in {source_path}')
    if config.plot_history_ms <= 0 or config.pose_history_length <= 0:
        raise ValueError(
            f'plot_history_ms and pose_history_length must be positive in '
            f'{source_path}'
        )
    if config.plot_grid_m <= 0.0:
        raise ValueError(f'plot_grid_m must be positive in {source_path}')
    if (
        config.plot_min_x >= config.plot_max_x
        or config.plot_min_y >= config.plot_max_y
    ):
        raise ValueError(f'Plot limits are invalid in {source_path}')
    if not 0.0 < config.confidence_threshold <= 1.0:
        raise ValueError(
            f'confidence_threshold must be in (0, 1] in {source_path}'
        )
    if not 0.0 < config.horizontal_fov_rad <= 2.0 * math.pi:
        raise ValueError(
            f'horizontal_fov_rad must be in (0, 2*pi] in {source_path}'
        )

    positive_values = {
        name: value
        for name, value in vars(config).items()
        if name in {
            'max_detection_distance_m',
            'max_pose_time_error_seconds',
            'multi_camera_merge_distance_m',
            'max_camera_batch_wait_seconds',
            'distance_log_scale',
            'distance_log_denominator',
            'bearing_log_scale',
            'bearing_log_denominator',
            'tracker_default_dt_seconds',
            'tracker_mahalanobis_gate',
            'tracker_max_position_distance_m',
            'tracker_max_prediction_step_seconds',
            'tracker_acceleration_noise_std',
            'tracker_measurement_noise_std',
            'tracker_initial_velocity_std',
            'tracker_reacquisition_mahalanobis_gate',
            'tracker_reacquisition_distance_m',
            'tracker_diagnostic_log_period_seconds',
        }
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be positive in {source_path}')

    if config.tracker_max_lost_frames < 1:
        raise ValueError(f'max_lost_frames must be positive in {source_path}')
    if config.tracker_min_confirmation_hits < 1:
        raise ValueError(
            f'min_confirmation_hits must be positive in {source_path}'
        )
    if (
        config.tracker_reacquisition_mahalanobis_gate
        < config.tracker_mahalanobis_gate
    ):
        raise ValueError(
            'reacquisition_mahalanobis_gate must be at least '
            f'mahalanobis_gate in {source_path}'
        )
    if (
        config.tracker_reacquisition_distance_m
        < config.tracker_max_position_distance_m
    ):
        raise ValueError(
            'reacquisition_distance_m must be at least '
            f'max_position_distance_m in {source_path}'
        )

    return config
