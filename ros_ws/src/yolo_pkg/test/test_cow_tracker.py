import math
import unittest
from types import SimpleNamespace

import numpy as np

from yolo_pkg.cow_tracker import CowTrackerManager
from yolo_pkg.cow_tracker_with_kalman_filter import (
    CowTrackerManager as KalmanCowTrackerManager,
)
from yolo_pkg.tracker_factory import create_cow_tracker


def make_tracker(max_lost_frames=3, min_hits=1, use_kalman_filter=False):
    return create_cow_tracker(SimpleNamespace(
        tracker_use_kalman_filter=use_kalman_filter,
        tracker_default_dt_seconds=0.1,
        tracker_max_lost_frames=max_lost_frames,
        tracker_mahalanobis_gate=9.21,
        tracker_max_position_distance_m=3.0,
        tracker_min_confirmation_hits=min_hits,
        tracker_max_prediction_step_seconds=1.0,
        tracker_acceleration_noise_std=1.5,
        tracker_measurement_noise_std=1.5,
        tracker_initial_velocity_std=3.0,
        tracker_reacquisition_mahalanobis_gate=16.0,
        tracker_reacquisition_distance_m=5.0,
    ))


def camera(yaw=0.0, horizontal_fov=1.0):
    return {
        'x': 0.0,
        'y': 0.0,
        'yaw': yaw,
        'horizontal_fov': horizontal_fov,
        'max_range': 20.0,
    }


class CowTrackerMemoryTests:
    def make_tracker(self, **options):
        return make_tracker(use_kalman_filter=self.use_kalman_filter, **options)

    def test_three_cows_keep_unseen_cow_at_last_observation(self):
        tracker = self.make_tracker(min_hits=3)
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
        remembered_x = min(tracks, key=lambda track: track['x'])['x']
        forgotten = next(
            track for track in tracker.tracks
            if track.track_id == forgotten_id
        )
        if self.use_kalman_filter:
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
            self.assertEqual(remembered['x'], remembered_x)
            self.assertEqual(remembered['y'], 0.0)
            self.assertTrue(remembered['coasting'])
            self.assertEqual(remembered['lost_frames'], 0)
            if self.use_kalman_filter:
                np.testing.assert_array_equal(forgotten.kf.x, frozen_state)
                np.testing.assert_array_equal(forgotten.kf.P, frozen_covariance)

        tracker.update([], timestamp=4.0, camera_views=[])
        if self.use_kalman_filter:
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

    def test_confirmed_cow_survives_visible_misses_past_expiry_limit(self):
        tracker = self.make_tracker(max_lost_frames=15)
        tracker.update([(2.0, 0.0)], timestamp=0.0, camera_views=[camera()])
        tracks = tracker.update(
            [(2.4, 0.0)], timestamp=0.1, camera_views=[camera()]
        )
        cow_id = tracks[0]['id']
        remembered_x = tracks[0]['x']

        for missed_frame in range(1, 151):
            tracks = tracker.update(
                [], timestamp=0.1 + missed_frame * 0.1,
                camera_views=[camera()],
            )
            self.assertEqual(len(tracks), 1)
            self.assertEqual(tracks[0]['id'], cow_id)
            self.assertEqual(tracks[0]['x'], remembered_x)
            self.assertEqual(tracks[0]['lost_frames'], missed_frame)
            self.assertEqual(tracker.last_diagnostics['removed_tracks'], 0)

        # Returning to the cow updates the same global map entry, not a new ID.
        tracks = tracker.update([(2.8, 0.0)], timestamp=15.2, camera_views=[camera()])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['id'], cow_id)
        self.assertFalse(tracks[0]['coasting'])
        self.assertEqual(tracks[0]['lost_frames'], 0)

    def test_cow_outside_local_window_keeps_real_global_position(self):
        tracker = self.make_tracker(max_lost_frames=3, min_hits=3)
        for step in range(3):
            tracks = tracker.update([(15.0, 2.0)], timestamp=step * 0.1,
                                    camera_views=[camera()])
        cow_id = tracks[0]['id']
        # The 20 m local window extends only +/-10 m, while the detector can
        # see 20 m. Misses here used to delete a known off-local-map cow.
        for step in range(3, 33):
            tracks = tracker.update([], timestamp=step * 0.1, camera_views=[camera()])
            self.assertEqual(len(tracks), 1)
            self.assertEqual((tracks[0]['id'], tracks[0]['x'], tracks[0]['y']),
                             (cow_id, 15.0, 2.0))
        # Searching elsewhere must neither shift the cow nor age it out.
        distant_camera = dict(camera(), x=-30.0, y=25.0)
        for step in range(33, 133):
            tracks = tracker.update([], timestamp=step * 0.1, camera_views=[distant_camera])
            self.assertEqual((tracks[0]['id'], tracks[0]['x'], tracks[0]['y']),
                             (cow_id, 15.0, 2.0))

    def test_redetection_resets_visible_misses(self):
        tracker = self.make_tracker(max_lost_frames=15)
        tracks = tracker.update(
            [(2.4, 0.0)], timestamp=0.0, camera_views=[camera()]
        )
        cow_id = tracks[0]['id']
        for step in range(1, 15):
            tracker.update([], timestamp=step * 0.1, camera_views=[camera()])
        tracks = tracker.update(
            [(2.8, 0.0)], timestamp=1.5, camera_views=[camera()]
        )
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]['id'], cow_id)
        self.assertEqual(tracks[0]['x'], 2.8)
        self.assertEqual(tracks[0]['lost_frames'], 0)

    def test_split_group_cannot_reassign_cow_outside_camera_view(self):
        tracker = self.make_tracker()
        tracks = tracker.update(
            [(4.0, 3.0), (4.0, 0.0)], timestamp=0.0,
            camera_views=[camera(horizontal_fov=math.pi)],
        )
        remembered_id = tracks[0]['id']

        # A new detection is within the wide reacquisition gate of the cow
        # left behind, but that cow's remembered position is out of view.
        for step in range(1, 31):
            tracks = tracker.update(
                [(4.0, 0.0), (5.0, 0.0)], timestamp=step * 0.1,
                camera_views=[camera()],
            )
            remembered = next(t for t in tracks if t['id'] == remembered_id)
            self.assertEqual((remembered['x'], remembered['y']), (4.0, 3.0))
            self.assertEqual(remembered['lost_frames'], 0)
            self.assertTrue(remembered['coasting'])
            self.assertEqual(len(tracks), 3)

        tracks = tracker.update(
            [(4.1, 3.0)], timestamp=3.1,
            camera_views=[camera(yaw=math.atan2(3.0, 4.0))],
        )
        remembered = next(t for t in tracks if t['id'] == remembered_id)
        self.assertEqual((remembered['x'], remembered['y']), (4.1, 3.0))
        self.assertFalse(remembered['coasting'])

    def test_tentative_cow_confirms_across_short_detection_gaps(self):
        tracker = self.make_tracker(max_lost_frames=3, min_hits=3)
        for step in range(4):
            tracker.update(
                [(2.0, 0.0)] if step % 2 == 0 else [],
                timestamp=step * 0.1, camera_views=[camera()],
            )
        tracks = tracker.update(
            [(2.0, 0.0)], timestamp=0.4, camera_views=[camera()]
        )
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracker.tracks[0].hits, 3)

    def test_tentative_cow_expires_after_visible_miss_limit(self):
        tracker = self.make_tracker(max_lost_frames=3, min_hits=3)
        tracker.update([(2.0, 0.0)], timestamp=0.0, camera_views=[camera()])
        for step in range(1, 3):
            tracker.update([], timestamp=step * 0.1, camera_views=[camera()])
            self.assertEqual(len(tracker.tracks), 1)
        tracker.update([], timestamp=0.3, camera_views=[camera()])
        self.assertEqual(tracker.tracks, [])
        self.assertEqual(tracker.last_diagnostics['removed_tracks'], 1)

    def test_out_of_view_misses_do_not_advance_existing_miss_count(self):
        tracker = self.make_tracker(max_lost_frames=3)
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


class UnfilteredCowTrackerTests(CowTrackerMemoryTests, unittest.TestCase):
    use_kalman_filter = False

    def test_matches_publish_exact_measurements_without_filter_state(self):
        tracker = self.make_tracker()
        self.assertIsInstance(tracker, CowTrackerManager)
        cow_id = None
        for step, position in enumerate([(2.0, 0.0), (2.8, 0.2), (2.1, -0.1)]):
            tracks = tracker.update(
                [position], timestamp=step * 0.1, camera_views=[camera()]
            )
            self.assertEqual(len(tracks), 1)
            self.assertEqual((tracks[0]['x'], tracks[0]['y']), position)
            self.assertEqual((tracks[0]['vx'], tracks[0]['vy']), (0.0, 0.0))
            if cow_id is None:
                cow_id = tracks[0]['id']
            self.assertEqual(tracks[0]['id'], cow_id)
            self.assertFalse(hasattr(tracker.tracks[0], 'kf'))

    def test_reordered_detections_keep_separate_map_entries(self):
        tracker = self.make_tracker()
        original = tracker.update(
            [(2.0, 0.0), (6.0, 0.0)], camera_views=[camera()]
        )
        tracks = tracker.update(
            [(6.2, 0.0), (2.2, 0.0)], camera_views=[camera()]
        )
        positions = {t['id']: t['x'] for t in tracks}
        self.assertEqual(positions, {
            original[0]['id']: 2.2, original[1]['id']: 6.2,
        })


class KalmanCowTrackerTests(CowTrackerMemoryTests, unittest.TestCase):
    use_kalman_filter = True

    def test_matches_still_apply_kalman_smoothing(self):
        tracker = self.make_tracker()
        self.assertIsInstance(tracker, KalmanCowTrackerManager)
        tracker.update([(2.0, 0.0)], timestamp=0.0, camera_views=[camera()])
        tracks = tracker.update(
            [(2.8, 0.0)], timestamp=0.1, camera_views=[camera()]
        )
        self.assertEqual(len(tracks), 1)
        self.assertGreater(tracks[0]['x'], 2.0)
        self.assertLess(tracks[0]['x'], 2.8)
        self.assertGreater(tracks[0]['vx'], 0.0)
