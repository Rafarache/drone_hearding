import itertools

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

        self.tracks = []
        self.last_timestamp = None

    def update(self, global_measurements, timestamp=None):
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

        if self.tracks and len(measurements) > 0:
            cost_matrix = self._build_cost_matrix(measurements)
            row_ind, col_ind = self._assign(cost_matrix)

            for track_idx, meas_idx in zip(row_ind, col_ind):
                if cost_matrix[track_idx, meas_idx] <= self.mahalanobis_gate:
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

        for track_idx, track in enumerate(self.tracks):
            if track_idx not in matched_track_indices:
                track.lost_frames += 1
                track.hit_streak = 0
                track.innovation_norm = float('nan')

        self.tracks = [
            track
            for track in self.tracks
            if (
                track.lost_frames < self.max_lost_frames
                if track.confirmed
                else track.lost_frames == 0
            )
        ]

        for meas_idx, measurement in enumerate(measurements):
            if meas_idx not in matched_meas_indices:
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

        return [
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

    def _build_cost_matrix(self, measurements):
        forbidden_cost = 1.0e9
        cost_matrix = np.full(
            (len(self.tracks), len(measurements)),
            fill_value=forbidden_cost,
            dtype=float,
        )

        for track_idx, track in enumerate(self.tracks):
            for meas_idx, measurement in enumerate(measurements):
                distance_sq = track.kf.mahalanobis_distance_sq(measurement)
                predicted_position = track.kf.x[:2, 0]
                position_distance = np.linalg.norm(
                    measurement - predicted_position
                )
                if (
                    distance_sq <= self.mahalanobis_gate
                    and position_distance <= self.max_position_distance
                ):
                    cost_matrix[track_idx, meas_idx] = distance_sq

        return cost_matrix

    @staticmethod
    def _assign(cost_matrix):
        return linear_sum_assignment(cost_matrix)
