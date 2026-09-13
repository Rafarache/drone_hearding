import itertools
import math

import numpy as np

from scipy.optimize import linear_sum_assignment


class CowKalmanFilter:
    def __init__(
        self,
        dt,
        initial_x,
        initial_y,
        acceleration_noise_std=1.5,
        measurement_noise_std=1.5,
        initial_velocity_std=3.0,
    ):
        self.dt = float(dt)
        acceleration_noise_std = float(acceleration_noise_std)
        measurement_noise_std = float(measurement_noise_std)
        initial_velocity_std = float(initial_velocity_std)
        if acceleration_noise_std <= 0.0:
            raise ValueError('acceleration_noise_std must be positive')
        if measurement_noise_std <= 0.0:
            raise ValueError('measurement_noise_std must be positive')
        if initial_velocity_std <= 0.0:
            raise ValueError('initial_velocity_std must be positive')

        self.acceleration_noise_variance = acceleration_noise_std ** 2
        self.x = np.array([[initial_x], [initial_y], [0.0], [0.0]], dtype=float)
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        measurement_variance = measurement_noise_std ** 2
        self.P = np.diag(
            [
                measurement_variance,
                measurement_variance,
                initial_velocity_std ** 2,
                initial_velocity_std ** 2,
            ]
        ).astype(float)
        self.R = np.eye(2, dtype=float) * measurement_variance
        self._set_dt(self.dt)

    def _set_dt(self, dt):
        self.dt = max(float(dt), 1.0e-3)
        self.F = np.array(
            [
                [1.0, 0.0, self.dt, 0.0],
                [0.0, 1.0, 0.0, self.dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        dt2 = self.dt * self.dt
        dt3 = dt2 * self.dt
        dt4 = dt2 * dt2
        q = self.acceleration_noise_variance
        self.Q = q * np.array(
            [
                [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
                [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
                [dt3 / 2.0, 0.0, dt2, 0.0],
                [0.0, dt3 / 2.0, 0.0, dt2],
            ],
            dtype=float,
        )

    def predict(self, dt=None):
        if dt is not None:
            self._set_dt(dt)

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)
        return self.x[0, 0], self.x[1, 0]

    def innovation(self, measurement):
        z = np.asarray(measurement, dtype=float).reshape(2, 1)
        residual = z - self.H @ self.x
        covariance = self.H @ self.P @ self.H.T + self.R
        return residual, covariance

    def mahalanobis_distance_sq(self, measurement):
        residual, covariance = self.innovation(measurement)
        return float((residual.T @ np.linalg.solve(covariance, residual)).item())

    def update(self, measurement_x, measurement_y):
        z = np.array([[measurement_x], [measurement_y]], dtype=float)
        residual, covariance = self.innovation(z)
        kalman_gain = np.linalg.solve(covariance.T, (self.P @ self.H.T).T).T

        self.x = self.x + kalman_gain @ residual

        identity = np.eye(self.P.shape[0], dtype=float)
        residual_gain = identity - kalman_gain @ self.H
        self.P = (
            residual_gain @ self.P @ residual_gain.T
            + kalman_gain @ self.R @ kalman_gain.T
        )
        self.P = 0.5 * (self.P + self.P.T)


class Track:
    _id_counter = itertools.count(1)

    def __init__(
        self,
        dt,
        initial_x,
        initial_y,
        min_hits,
        acceleration_noise_std,
        measurement_noise_std,
        initial_velocity_std,
    ):
        self.track_id = next(Track._id_counter)
        self.kf = CowKalmanFilter(
            dt,
            initial_x,
            initial_y,
            acceleration_noise_std=acceleration_noise_std,
            measurement_noise_std=measurement_noise_std,
            initial_velocity_std=initial_velocity_std,
        )
        self.min_hits = min_hits
        self.hits = 1
        self.hit_streak = 1
        self.age = 1
        self.lost_frames = 0
        self.is_confirmed = min_hits <= 1
        self.innovation_norm = float('nan')

    @property
    def confirmed(self):
        return self.is_confirmed

    @property
    def coasting(self):
        return self.lost_frames > 0


class CowTrackerManager:
    def __init__(
        self,
        dt=0.1,
        max_lost_frames=15,
        mahalanobis_gate=9.21,
        max_position_distance=3.0,
        min_hits=3,
        max_dt=1.0,
        acceleration_noise_std=1.5,
        measurement_noise_std=1.5,
        initial_velocity_std=3.0,
        reacquisition_mahalanobis_gate=16.0,
        reacquisition_distance=5.0,
    ):
        self.default_dt = float(dt)
        self.max_lost_frames = int(max_lost_frames)
        self.mahalanobis_gate = float(mahalanobis_gate)
        self.max_position_distance = float(max_position_distance)
        self.min_hits = int(min_hits)
        self.max_dt = float(max_dt)
        self.acceleration_noise_std = float(acceleration_noise_std)
        self.measurement_noise_std = float(measurement_noise_std)
        self.initial_velocity_std = float(initial_velocity_std)
        self.reacquisition_mahalanobis_gate = float(
            reacquisition_mahalanobis_gate
        )
        self.reacquisition_distance = float(reacquisition_distance)

        if self.default_dt <= 0.0:
            raise ValueError('dt must be positive')
        if self.max_lost_frames < 1:
            raise ValueError('max_lost_frames must be at least one')
        if self.mahalanobis_gate <= 0.0:
            raise ValueError('mahalanobis_gate must be positive')
        if self.max_position_distance <= 0.0:
            raise ValueError('max_position_distance must be positive')
        if self.min_hits < 1:
            raise ValueError('min_hits must be at least one')
        if self.max_dt <= 0.0:
            raise ValueError('max_dt must be positive')
        if self.acceleration_noise_std <= 0.0:
            raise ValueError('acceleration_noise_std must be positive')
        if self.measurement_noise_std <= 0.0:
            raise ValueError('measurement_noise_std must be positive')
        if self.initial_velocity_std <= 0.0:
            raise ValueError('initial_velocity_std must be positive')
        if self.reacquisition_mahalanobis_gate < self.mahalanobis_gate:
            raise ValueError(
                'reacquisition_mahalanobis_gate must be greater than or '
                'equal to mahalanobis_gate'
            )
        if self.reacquisition_distance < self.max_position_distance:
            raise ValueError(
                'reacquisition_distance must be greater than or equal to '
                'max_position_distance'
            )

        self.tracks = []
        self.last_timestamp = None
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
        dt = self._compute_dt(timestamp)
        measurements = np.asarray(global_measurements, dtype=float).reshape(-1, 2)
        measurements = measurements[np.all(np.isfinite(measurements), axis=1)]

        prediction_steps = max(1, int(np.ceil(dt / self.max_dt)))
        prediction_dt = dt / prediction_steps
        for track in self.tracks:
            for _ in range(prediction_steps):
                track.kf.predict(prediction_dt)
            track.age += 1

        matched_track_indices = set()
        matched_meas_indices = set()
        available_measurements = set(range(len(measurements)))
        confirmed_indices = {
            index
            for index, track in enumerate(self.tracks)
            if track.confirmed
        }
        tentative_indices = set(range(len(self.tracks))) - confirmed_indices

        regular_matches = self._associate(
            confirmed_indices,
            available_measurements,
            measurements,
            self.mahalanobis_gate,
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
            if self.tracks[index].lost_frames > 0
        }
        reacquisition_matches = self._associate(
            reacquisition_candidates,
            available_measurements,
            measurements,
            self.reacquisition_mahalanobis_gate,
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
            self.mahalanobis_gate,
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

            track.hit_streak = 0
            track.innovation_norm = float('nan')
            if self._is_position_visible(track.kf.x[:2, 0], camera_views):
                track.lost_frames += 1
                visible_misses += 1

        tracks_before_removal = len(self.tracks)
        self.tracks = [
            track
            for track in self.tracks
            if (
                track.lost_frames < self.max_lost_frames
                if track.confirmed
                else track.lost_frames == 0
            )
        ]
        removed_tracks = tracks_before_removal - len(self.tracks)

        for meas_idx in sorted(available_measurements):
            measurement = measurements[meas_idx]
            self.tracks.append(
                Track(
                    self.default_dt,
                    initial_x=measurement[0],
                    initial_y=measurement[1],
                    min_hits=self.min_hits,
                    acceleration_noise_std=self.acceleration_noise_std,
                    measurement_noise_std=self.measurement_noise_std,
                    initial_velocity_std=self.initial_velocity_std,
                )
            )

        confirmed_tracks = [
            {
                "id": track.track_id,
                "x": track.kf.x[0, 0],
                "y": track.kf.x[1, 0],
                "vx": track.kf.x[2, 0],
                "vy": track.kf.x[3, 0],
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

    def _compute_dt(self, timestamp):
        if timestamp is None:
            return self.default_dt

        timestamp = float(timestamp)
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return self.default_dt

        dt = timestamp - self.last_timestamp
        if dt <= 0.0:
            return 1.0e-3

        self.last_timestamp = timestamp
        return dt

    def _associate(
        self,
        track_indices,
        measurement_indices,
        measurements,
        mahalanobis_gate,
        max_position_distance,
    ):
        track_indices = sorted(track_indices)
        measurement_indices = sorted(measurement_indices)
        if not track_indices or not measurement_indices:
            return []

        cost_matrix = self._build_cost_matrix(
            track_indices,
            measurement_indices,
            measurements,
            mahalanobis_gate,
            max_position_distance,
        )
        row_ind, col_ind = self._assign(cost_matrix)
        matches = []
        for row, col in zip(row_ind, col_ind):
            if cost_matrix[row, col] <= mahalanobis_gate:
                matches.append(
                    (track_indices[row], measurement_indices[col])
                )
        return matches

    def _build_cost_matrix(
        self,
        track_indices,
        measurement_indices,
        measurements,
        mahalanobis_gate,
        max_position_distance,
    ):
        forbidden_cost = 1.0e9
        cost_matrix = np.full(
            (len(track_indices), len(measurement_indices)),
            fill_value=forbidden_cost,
            dtype=float,
        )

        for row, track_idx in enumerate(track_indices):
            track = self.tracks[track_idx]
            for col, meas_idx in enumerate(measurement_indices):
                measurement = measurements[meas_idx]
                distance_sq = track.kf.mahalanobis_distance_sq(measurement)
                predicted_position = track.kf.x[:2, 0]
                position_distance = np.linalg.norm(
                    measurement - predicted_position
                )
                if (
                    distance_sq <= mahalanobis_gate
                    and position_distance <= max_position_distance
                ):
                    cost_matrix[row, col] = distance_sq

        return cost_matrix

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
            innovation, _ = track.kf.innovation(measurement)
            track.innovation_norm = float(np.linalg.norm(innovation))
            track.kf.update(measurement[0], measurement[1])
            track.hits += 1
            track.hit_streak += 1
            track.lost_frames = 0
            if track.hit_streak >= track.min_hits:
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

    @staticmethod
    def _assign(cost_matrix):
        return linear_sum_assignment(cost_matrix)
