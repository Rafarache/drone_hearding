"""Camera footprint sampling, independent of detections and tracker batching."""

import math


class ObservationSampler:
    def __init__(self, near=2.0, far=10.0, fov=2.09, period=1.0):
        if not all(math.isfinite(v) for v in (near, far, fov, period)):
            raise ValueError('Observation settings must be finite')
        if not (0 <= near < 5 < far and 0 < fov <= 2 * math.pi and period > 0):
            raise ValueError('Invalid observation settings')
        self.near, self.far, self.fov, self.period = near, far, fov, period
        # Conservative forward rectangle, entirely inside the annular FOV.
        # Keep the 5 m search look-at point inside; mirrored in SearchCoverage.
        self.front = max(near, min(4.0, far * 0.4))
        self.back = max(5.0, (self.front + math.sqrt(self.front**2 + 8 * far**2)) / 4)
        self.half_width = math.sqrt(far**2 - self.back**2)
        if fov < math.pi:
            self.half_width = min(self.half_width, self.front * math.tan(fov / 2))
        self.last = {}

    def sample(self, source, timestamp, x, y, yaw):
        if not all(math.isfinite(v) for v in (timestamp, x, y, yaw)):
            return None
        previous = self.last.get(source)
        if previous is not None and 0 <= timestamp - previous < self.period - 1e-9:
            return None
        self.last[source] = timestamp
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return [(x + forward * cosine - lateral * sine,
                 y + forward * sine + lateral * cosine)
                for forward, lateral in ((self.front, -self.half_width),
                                         (self.back, -self.half_width),
                                         (self.back, self.half_width),
                                         (self.front, self.half_width))]
