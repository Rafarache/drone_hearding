"""Conservative empty-observation evidence, separate from association gates."""

import math


def observed_camera_views(position, camera_views):
    """Keep views whose processed ground footprint contains this position.

    Legacy callers can supply an explicit FOV/range without a polygon. The
    caller still applies its FOV/range check to the returned views. No camera
    metadata means no evidence that a remembered cow's location was inspected.
    """
    result = []
    x, y = map(float, position)
    if not math.isfinite(x) or not math.isfinite(y):
        return result
    for camera in camera_views or []:
        polygon = camera.get('observed_polygon')
        if polygon is None:
            result.append(camera)
            continue
        if len(polygon) < 3 or not all(
            math.isfinite(value) for point in polygon for value in point
        ):
            continue
        crosses = []
        area = 0.0
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            crosses.append((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]))
            area += a[0] * b[1] - a[1] * b[0]
        if abs(area) > 1e-9 and (
            all(cross >= -1e-9 for cross in crosses)
            or all(cross <= 1e-9 for cross in crosses)
        ):
            result.append(camera)
    return result
