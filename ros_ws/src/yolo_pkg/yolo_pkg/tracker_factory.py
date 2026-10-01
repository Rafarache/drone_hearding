"""Select the cow tracker used by the simulation."""

from yolo_pkg.cow_tracker import CowTrackerManager


def create_cow_tracker(config):
    options = dict(
        max_lost_frames=config.tracker_max_lost_frames,
        max_position_distance=config.tracker_max_position_distance_m,
        min_hits=config.tracker_min_confirmation_hits,
        reacquisition_distance=config.tracker_reacquisition_distance_m,
    )
    if not config.tracker_use_kalman_filter:
        return CowTrackerManager(**options)

    from yolo_pkg.cow_tracker_with_kalman_filter import (
        CowTrackerManager as KalmanCowTrackerManager,
    )

    return KalmanCowTrackerManager(
        **options,
        dt=config.tracker_default_dt_seconds,
        mahalanobis_gate=config.tracker_mahalanobis_gate,
        max_dt=config.tracker_max_prediction_step_seconds,
        acceleration_noise_std=config.tracker_acceleration_noise_std,
        measurement_noise_std=config.tracker_measurement_noise_std,
        initial_velocity_std=config.tracker_initial_velocity_std,
        reacquisition_mahalanobis_gate=(
            config.tracker_reacquisition_mahalanobis_gate
        ),
    )
