import itertools

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    linear_sum_assignment = None


class CowKalmanFilter:
    def __init__(
        self,
        dt,
        initial_x,
        initial_y,
        acceleration_noise=2.0,
        measurement_noise_std=5.0,
    ):
        self.dt = float(dt)
        self.acceleration_noise = float(acceleration_noise)
        self.x = np.array([[initial_x], [initial_y], [0.0], [0.0]], dtype=float)
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        self.P = np.diag([25.0, 25.0, 100.0, 100.0]).astype(float)
        self.R = np.eye(2, dtype=float) * float(measurement_noise_std) ** 2
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
        q = self.acceleration_noise
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

    def __init__(self, dt, initial_x, initial_y, min_hits):
        self.track_id = next(Track._id_counter)
        self.kf = CowKalmanFilter(dt, initial_x, initial_y)
        self.min_hits = min_hits
        self.hits = 1
        self.age = 1
        self.lost_frames = 0

    @property
    def confirmed(self):
        return self.hits >= self.min_hits

    @property
    def coasting(self):
        return self.lost_frames > 0


class CowTrackerManager:
    def __init__(
        self,
        dt=0.1,
        max_lost_frames=15,
        mahalanobis_gate=9.21,
        min_hits=2,
        max_dt=1.0,
    ):
        self.default_dt = float(dt)
        self.max_lost_frames = int(max_lost_frames)
        self.mahalanobis_gate = float(mahalanobis_gate)
        self.min_hits = int(min_hits)
        self.max_dt = float(max_dt)
        self.tracks = []
        self.last_timestamp = None

    def update(self, global_measurements, timestamp=None):
        dt = self._compute_dt(timestamp)
        measurements = np.asarray(global_measurements, dtype=float).reshape(-1, 2)

        for track in self.tracks:
            track.kf.predict(dt)
            track.age += 1

        matched_track_indices = set()
        matched_meas_indices = set()

        if self.tracks and len(measurements) > 0:
            cost_matrix = self._build_cost_matrix(measurements)
            row_ind, col_ind = self._assign(cost_matrix)

            for track_idx, meas_idx in zip(row_ind, col_ind):
                if cost_matrix[track_idx, meas_idx] <= self.mahalanobis_gate:
                    measurement = measurements[meas_idx]
                    self.tracks[track_idx].kf.update(measurement[0], measurement[1])
                    self.tracks[track_idx].hits += 1
                    self.tracks[track_idx].lost_frames = 0
                    matched_track_indices.add(track_idx)
                    matched_meas_indices.add(meas_idx)

        for track_idx, track in enumerate(self.tracks):
            if track_idx not in matched_track_indices:
                track.lost_frames += 1

        self.tracks = [
            track for track in self.tracks
            if track.lost_frames < self.max_lost_frames
        ]

        for meas_idx, measurement in enumerate(measurements):
            if meas_idx not in matched_meas_indices:
                self.tracks.append(
                    Track(
                        self.default_dt,
                        initial_x=measurement[0],
                        initial_y=measurement[1],
                        min_hits=self.min_hits,
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
            return self.default_dt

        self.last_timestamp = timestamp
        return min(dt, self.max_dt)

    def _build_cost_matrix(self, measurements):
        cost_matrix = np.full(
            (len(self.tracks), len(measurements)),
            fill_value=self.mahalanobis_gate + 1.0,
            dtype=float,
        )

        for track_idx, track in enumerate(self.tracks):
            for meas_idx, measurement in enumerate(measurements):
                distance_sq = track.kf.mahalanobis_distance_sq(measurement)
                if distance_sq <= self.mahalanobis_gate:
                    cost_matrix[track_idx, meas_idx] = distance_sq

        return cost_matrix

    def _assign(self, cost_matrix):
        if linear_sum_assignment is not None:
            return linear_sum_assignment(cost_matrix)

        # Small fallback for environments where scipy is not installed.
        pairs = []
        used_rows = set()
        used_cols = set()
        flat_indices = np.argsort(cost_matrix, axis=None)
        rows, cols = np.unravel_index(flat_indices, cost_matrix.shape)

        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue
            used_rows.add(row)
            used_cols.add(col)
            pairs.append((row, col))

        if not pairs:
            return np.array([], dtype=int), np.array([], dtype=int)

        row_ind, col_ind = zip(*pairs)
        return np.asarray(row_ind), np.asarray(col_ind)
