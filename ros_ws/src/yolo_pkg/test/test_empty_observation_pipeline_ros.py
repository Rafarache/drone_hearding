"""Exercise real image/batch/publishing code without loading detector weights."""

from dataclasses import replace
from types import MethodType, SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Image

from yolo_pkg.config import load_yolo_config
from yolo_pkg.search_observation import ObservationSampler
from yolo_pkg.tracker_factory import create_cow_tracker
from yolo_pkg.yolo_subscriber import YoloCowSubscriber


def pipeline(use_kalman_filter):
    config = replace(load_yolo_config(), tracker_use_kalman_filter=use_kalman_filter,
                     tracker_max_lost_frames=3, tracker_min_confirmation_hits=1)
    tracks, observations = [], []
    fake = SimpleNamespace(
        config=config, observation_sampler=ObservationSampler(),
        camera_horizontal_fov_rad=config.horizontal_fov_rad,
        show_annotated_camera=False, pending_batch_start_timestamp=None,
        pending_detection_batches={}, expected_camera_names={'drone0'},
        drone_index_by_name={'drone0': 0}, plotter=None,
        observation_publishers={'drone0': SimpleNamespace(publish=observations.append)},
        publisher=SimpleNamespace(publish=tracks.append),
        bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.zeros((480, 640, 3))),
        model=lambda *a, **k: [SimpleNamespace(boxes=[])],
        _drone_state_at=lambda *a: (0.0, 0.0, 0.0),
        _log_tracker_diagnostics=lambda *a: None,
        _stamp_to_seconds=YoloCowSubscriber._stamp_to_seconds,
        tracker=create_cow_tracker(config),
    )
    for method in ('_flush_stale_detection_batch', '_process_detection_batch',
                   '_merge_camera_measurements', '_publish_tracks', 'listener_callback'):
        setattr(fake, method, MethodType(getattr(YoloCowSubscriber, method), fake))
    fake.tracker.update([(5.0, 0.0)])
    return fake, tracks, observations


@pytest.mark.parametrize('use_kalman_filter', [False, True])
def test_empty_images_clear_confirmed_tracks_and_publish_empty_map(use_kalman_filter):
    fake, tracks, observations = pipeline(use_kalman_filter)
    for frame in range(1, 4):
        image = Image()
        image.header.stamp.nanosec = frame * 100_000_000
        fake.listener_callback(image, 'drone0')
        assert len(tracks[-1].poses) == (1 if frame < 3 else 0)
        assert fake.pending_detection_batches == {}
    assert len(observations) == 1  # Coverage is throttled; negative evidence is not.
    assert len(observations[0].polygon.points) == 4
    assert tracks[-1].header.frame_id == 'map'
    assert fake.tracker.last_diagnostics['removed_tracks'] == 1


@pytest.mark.parametrize('use_kalman_filter', [False, True])
def test_missing_pose_or_failed_processing_is_not_empty_evidence(use_kalman_filter):
    fake, tracks, observations = pipeline(use_kalman_filter)
    fake._drone_state_at = lambda *a: None
    for _ in range(4):
        fake.listener_callback(Image(), 'drone0')
    assert not tracks and not observations
    fake._drone_state_at = lambda *a: (0.0, 0.0, 0.0)

    def failed_model(*args, **kwargs):
        raise RuntimeError('inference failed')

    fake.model = failed_model
    for _ in range(4):
        with pytest.raises(RuntimeError, match='inference failed'):
            fake.listener_callback(Image(), 'drone0')
    assert not tracks and not observations
    assert len(fake.tracker.tracks) == 1
    assert fake.tracker.tracks[0].lost_frames == 0
