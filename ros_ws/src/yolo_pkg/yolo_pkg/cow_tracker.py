"""Cow map memory using direct measurements, without motion filtering."""

import itertools
import math

import numpy as np

from scipy.optimize import linear_sum_assignment


class Track:
    _id_counter = itertools.count(1)

    def __init__(
        self,
        initial_x,
        initial_y,
        min_hits,
    ):
        self.track_id = next(Track._id_counter)
        self.min_hits = min_hits
        self.hits = 1
        self.age = 1
        self.lost_frames = 0
        self.is_confirmed = min_hits <= 1
        self.innovation_norm = float('nan')
        # Store measurements exactly, without smoothing or prediction.
        self.last_observed_position = np.array(
            [initial_x, initial_y], dtype=float
        )
        self.unobserved = False

    @property
    def confirmed(self):
        return self.is_confirmed

    @property
    def coasting(self):
        return self.unobserved


class CowTrackerManager:
    def __init__(
        self,
        max_lost_frames,
        max_position_distance,
        min_hits,
        reacquisition_distance,
    ):
        self.max_lost_frames = int(max_lost_frames)
        self.max_position_distance = float(max_position_distance)
        self.min_hits = int(min_hits)
        self.reacquisition_distance = float(reacquisition_distance)
        if self.max_lost_frames < 1:
            raise ValueError('max_lost_frames must be at least one')
        if self.min_hits < 1:
            raise ValueError('min_hits must be at least one')
        if (
            not math.isfinite(self.max_position_distance)
            or self.max_position_distance <= 0
        ):
            raise ValueError('max_position_distance must be finite and positive')
        if (
            not math.isfinite(self.reacquisition_distance)
            or self.reacquisition_distance < self.max_position_distance
        ):
            raise ValueError(
                'reacquisition_distance must be finite and '
                '>= max_position_distance'
            )

        self.tracks = []
        self.last_diagnostics = {
            'measurements': 0,
            'total_tracks': 0,
            'confirmed_tracks': 0,
            'coasting_tracks': 0,
            'visible_misses': 0,
            'reacquisitions': 0,
            'removed_tracks': 0,
        }

    def update(self, global_measurements, timestamp=None, camera_views=None):
        # timestamp is accepted for the same update API as the filtered tracker.
        measurements = np.asarray(global_measurements, dtype=float).reshape(-1, 2)
        measurements = measurements[np.all(np.isfinite(measurements), axis=1)]

        for track in self.tracks:
            track.age += 1
            if (
                track.unobserved
                or not self._is_position_visible(
                    track.last_observed_position, camera_views
                )
            ):
                track.unobserved = True

        matched_track_indices = set()
        matched_meas_indices = set()
        available_measurements = set(range(len(measurements)))
        # A nearby detection must not steal an identity remembered outside
        # the cameras' view. None supports callers without camera metadata;
        # an explicit empty list means no camera can currently see a track.
        observable_indices = {
            index for index, track in enumerate(self.tracks)
            if camera_views is None or self._is_position_visible(
                track.last_observed_position, camera_views
            )
        }
        confirmed_indices = {
            index
            for index, track in enumerate(self.tracks)
            if track.confirmed and index in observable_indices
        }
        tentative_indices = observable_indices - confirmed_indices
        active_confirmed_indices = {
            index for index in confirmed_indices
            if not self.tracks[index].unobserved
        }

        regular_matches = self._associate(
            active_confirmed_indices,
            available_measurements,
            measurements,
            self.max_position_distance,
        )
        self._apply_matches(
            regular_matches,
            measurements,
            matched_track_indices,
            matched_meas_indices,
        )
        available_measurements -= matched_meas_indices

        # Give established, temporarily lost identities first access to a
        # wider gate before tentative tracks can claim the measurement.
        reacquisition_candidates = {
            index
            for index in confirmed_indices - matched_track_indices
            if self.tracks[index].unobserved
        }
        reacquisition_matches = self._associate(
            reacquisition_candidates,
            available_measurements,
            measurements,
            self.reacquisition_distance,
        )
        self._apply_matches(
            reacquisition_matches,
            measurements,
            matched_track_indices,
            matched_meas_indices,
        )
        available_measurements -= matched_meas_indices

        tentative_matches = self._associate(
            tentative_indices,
            available_measurements,
            measurements,
            self.max_position_distance,
        )
        self._apply_matches(
            tentative_matches,
            measurements,
            matched_track_indices,
            matched_meas_indices,
        )
        available_measurements -= matched_meas_indices

        visible_misses = 0
        for track_idx, track in enumerate(self.tracks):
            if track_idx in matched_track_indices:
                continue

            track.innovation_norm = float('nan')
            track.unobserved = True
            if self._is_position_visible(
                track.last_observed_position, camera_views
            ):
                track.lost_frames += 1
                visible_misses += 1

        tracks_before_removal = len(self.tracks)
        # Confirmed cows are mission memory, not temporary camera tracks.
        # Keep their last global position until a detection updates it. Only
        # unconfirmed candidates can expire after counted visible misses.
        self.tracks = [
            track
            for track in self.tracks
            if track.confirmed or track.lost_frames < self.max_lost_frames
        ]
        removed_tracks = tracks_before_removal - len(self.tracks)

        for meas_idx in sorted(available_measurements):
            measurement = measurements[meas_idx]
            self.tracks.append(
                Track(
                    initial_x=measurement[0],
                    initial_y=measurement[1],
                    min_hits=self.min_hits,
                )
            )

        confirmed_tracks = [
            {
                "id": track.track_id,
                "x": track.last_observed_position[0],
                "y": track.last_observed_position[1],
                "vx": 0.0,
                "vy": 0.0,
                "coasting": track.coasting,
                "confirmed": track.confirmed,
                "lost_frames": track.lost_frames,
                "innovation_norm": track.innovation_norm,
            }
            for track in self.tracks
            if track.confirmed
        ]
        self.last_diagnostics = {
            'measurements': len(measurements),
            'total_tracks': len(self.tracks),
            'confirmed_tracks': len(confirmed_tracks),
            'coasting_tracks': sum(
                track.coasting for track in self.tracks if track.confirmed
            ),
            'visible_misses': visible_misses,
            'reacquisitions': len(reacquisition_matches),
            'removed_tracks': removed_tracks,
        }
        return confirmed_tracks

    def _associate(
        self,
        track_indices,
        measurement_indices,
        measurements,
        max_position_distance,
    ):
        track_indices = sorted(track_indices)
        measurement_indices = sorted(measurement_indices)
        if not track_indices or not measurement_indices:
            return []

        # One-to-one nearest-position assignment within a metric gate.
        cost_matrix = np.full(
            (len(track_indices), len(measurement_indices)), 1.0e9
        )
        for row, track_idx in enumerate(track_indices):
            position = self.tracks[track_idx].last_observed_position
            for col, meas_idx in enumerate(measurement_indices):
                distance = np.linalg.norm(measurements[meas_idx] - position)
                if distance <= max_position_distance:
                    cost_matrix[row, col] = distance
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return [
            (track_indices[row], measurement_indices[col])
            for row, col in zip(row_ind, col_ind)
            if cost_matrix[row, col] <= max_position_distance
        ]

    def _apply_matches(
        self,
        matches,
        measurements,
        matched_track_indices,
        matched_meas_indices,
    ):
        for track_idx, meas_idx in matches:
            measurement = measurements[meas_idx]
            track = self.tracks[track_idx]
            track.innovation_norm = float(np.linalg.norm(
                measurement - track.last_observed_position
            ))
            track.last_observed_position = measurement.copy()
            track.unobserved = False
            track.hits += 1
            track.lost_frames = 0
            if track.hits >= track.min_hits:
                track.is_confirmed = True
            matched_track_indices.add(track_idx)
            matched_meas_indices.add(meas_idx)

    @staticmethod
    def _is_position_visible(position, camera_views):
        if not camera_views:
            return False

        position_x = float(position[0])
        position_y = float(position[1])
        if not np.isfinite(position_x) or not np.isfinite(position_y):
            return False

        for camera in camera_views:
            camera_x = float(camera['x'])
            camera_y = float(camera['y'])
            camera_yaw = float(camera['yaw'])
            horizontal_fov = float(camera['horizontal_fov'])
            max_range = float(camera['max_range'])
            values = (
                camera_x,
                camera_y,
                camera_yaw,
                horizontal_fov,
                max_range,
            )
            if not all(np.isfinite(value) for value in values):
                continue

            delta_x = position_x - camera_x
            delta_y = position_y - camera_y
            if math.hypot(delta_x, delta_y) > max_range:
                continue

            bearing_error = (
                math.atan2(delta_y, delta_x) - camera_yaw + math.pi
            ) % (2.0 * math.pi) - math.pi
            if abs(bearing_error) <= horizontal_fov / 2.0:
                return True

        return False
