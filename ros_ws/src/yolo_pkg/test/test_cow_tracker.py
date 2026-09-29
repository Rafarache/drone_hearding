import math
import unittest

import numpy as np

from yolo_pkg.cow_tracker import CowTrackerManager


def make_tracker(max_lost_frames=3, min_hits=1):
    return CowTrackerManager(
        dt=0.1,
        max_lost_frames=max_lost_frames,
        mahalanobis_gate=9.21,
        max_position_distance=3.0,
        min_hits=min_hits,
        max_dt=1.0,
        acceleration_noise_std=1.5,
        measurement_noise_std=1.5,
        initial_velocity_std=3.0,
        reacquisition_mahalanobis_gate=16.0,
        reacquisition_distance=5.0,
    )


def camera(yaw=0.0, horizontal_fov=1.0):
    return {
        'x': 0.0,
        'y': 0.0,
        'yaw': yaw,
        'horizontal_fov': horizontal_fov,
        'max_range': 20.0,
    }


class CowTrackerMemoryTests(unittest.TestCase):
    def test_three_cows_keep_unseen_cow_at_last_observation(self):
        tracker = make_tracker(min_hits=3)
        tracker.update(
            [(-10.0, 0.0), (1.0, 0.0), (5.0, 0.0)],
            timestamp=0.0,
            camera_views=[camera(horizontal_fov=2.0 * math.pi)],
        )
        tracker.update(
            [(-9.8, 0.0), (1.2, 0.0), (5.2, 0.0)],
            timestamp=0.1,
            camera_views=[camera(horizontal_fov=2.0 * math.pi)],
        )
        tracks = tracker.update(
            [(-9.6, 0.0), (1.4, 0.0), (5.4, 0.0)],
            timestamp=0.2,
            camera_views=[camera(horizontal_fov=2.0 * math.pi)],
        )
        self.assertEqual(len(tracks), 3)
        forgotten_id = min(tracks, key=lambda track: track['x'])['id']
        forgotten = next(
            track for track in tracker.tracks
            if track.track_id == forgotten_id
        )
        self.assertGreater(forgotten.kf.x[2, 0], 0.0)
        frozen_state = forgotten.kf.x.copy()
        frozen_covariance = forgotten.kf.P.copy()

        for step in range(1, 31):
            tracks = tracker.update(
                [(1.4, 0.0), (5.4, 0.0)],
                timestamp=0.2 + step * 0.1,
                camera_views=[camera()],
            )
            remembered = next(
                track for track in tracks if track['id'] == forgotten_id
            )
            self.assertEqual(len(tracks), 3)
            self.assertEqual(remembered['x'], -9.6)
            self.assertEqual(remembered['y'], 0.0)
            self.assertTrue(remembered['coasting'])
            self.assertEqual(remembered['lost_frames'], 0)
            np.testing.assert_array_equal(forgotten.kf.x, frozen_state)
            np.testing.assert_array_equal(forgotten.kf.P, frozen_covariance)

        tracker.update([], timestamp=4.0, camera_views=[])
        np.testing.assert_array_equal(forgotten.kf.x, frozen_state)
        np.testing.assert_array_equal(forgotten.kf.P, frozen_covariance)

        tracks = tracker.update(
            [(-9.2, 0.0)],
            timestamp=4.1,
            camera_views=[camera(yaw=math.pi)],
        )
        remembered = next(
            track for track in tracks if track['id'] == forgotten_id
        )
        self.assertEqual(len(tracks), 3)
        self.assertEqual(remembered['x'], -9.2)
        self.assertFalse(remembered['coasting'])

    def test_visible_misses_freeze_position_then_retire_track(self):
        tracker = make_tracker(max_lost_frames=3)
        tracker.update([(2.0, 0.0)], timestamp=0.0, camera_views=[camera()])
        tracks = tracker.update(
            [(2.4, 0.0)], timestamp=0.1, camera_views=[camera()]
        )
        cow_id = tracks[0]['id']

        for missed_frame in (1, 2):
            tracks = tracker.update(
                [], timestamp=0.1 + missed_frame * 0.1,
                camera_views=[camera()],
            )
            self.assertEqual(len(tracks), 1)
            self.assertEqual(tracks[0]['id'], cow_id)
            self.assertEqual(tracks[0]['x'], 2.4)
            self.assertEqual(tracks[0]['lost_frames'], missed_frame)

        tracks = tracker.update([], timestamp=0.4, camera_views=[camera()])
        self.assertEqual(tracks, [])
        self.assertEqual(tracker.last_diagnostics['removed_tracks'], 1)

    def test_out_of_view_misses_do_not_advance_existing_miss_count(self):
        tracker = make_tracker(max_lost_frames=3)
        tracker.update([(2.0, 0.0)], timestamp=0.0, camera_views=[camera()])
        tracker.update([], timestamp=0.1, camera_views=[camera()])

        for step in range(2, 22):
            tracks = tracker.update(
                [], timestamp=step * 0.1,
                camera_views=[camera(yaw=math.pi)],
            )
            self.assertEqual(len(tracks), 1)
            self.assertEqual(tracks[0]['x'], 2.0)
            self.assertEqual(tracks[0]['lost_frames'], 1)
