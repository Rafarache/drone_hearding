import math

import pytest

from yolo_pkg.search_observation import ObservationSampler


def test_sample_before_acceptance_and_per_camera_throttling():
    sampler = ObservationSampler()
    assert sampler.sample('a', 0, 0, 0, 0)
    assert sampler.sample('a', 0.1, 100, 100, 0) is None
    assert sampler.sample('b', 0.1, 0, 0, 0)
    assert sampler.sample('a', 0.2, 0, 0, 0) is None
    assert sampler.sample('a', 0.999, 0, 0, 0) is None
    assert sampler.sample('a', 1.0, 0, 0, 0)
    assert sampler.sample('a', -1, 0, 0, 0)  # simulation clock reset


@pytest.mark.parametrize('near,far,fov', [(2, 10, 2.09), (0, 6, 0.1),
                                       (4.9, 5.1, 1), (0, 20, 2 * math.pi)])
@pytest.mark.parametrize('yaw', [0, math.pi / 2, -2.3])
def test_box_geometry_and_rotation(near, far, fov, yaw):
    sampler = ObservationSampler(near, far, fov)
    points = sampler.sample('a', 0, 3, 4, yaw)
    assert len(points) == 4
    # Check the interior too: a bounding box around the cone would incorrectly
    # mark ground outside the camera range/FOV, including behind the drone.
    for i in range(11):
        for j in range(11):
            u, v = i / 10, j / 10
            x = points[0][0] + u * (points[1][0] - points[0][0]) + v * (points[3][0] - points[0][0])
            y = points[0][1] + u * (points[1][1] - points[0][1]) + v * (points[3][1] - points[0][1])
            radius = math.hypot(x - 3, y - 4)
            angle = math.atan2(y - 4, x - 3) - yaw
            assert near - 1e-9 <= radius <= far + 1e-9
            assert abs(math.atan2(math.sin(angle), math.cos(angle))) <= fov / 2 + 1e-9
    assert sampler.front <= 5 <= sampler.back
    assert sampler.sample('b', 0, float('nan'), 0, 0) is None


def test_custom_period():
    sampler = ObservationSampler(period=2.0)
    assert sampler.sample('a', 10, 0, 0, 0)
    assert sampler.sample('a', 11, 0, 0, 0) is None
    assert sampler.sample('a', 12, 0, 0, 0)


def test_invalid_settings():
    with pytest.raises(ValueError):
        ObservationSampler(near=6)
    with pytest.raises(ValueError):
        ObservationSampler(fov=float('nan'))
    with pytest.raises(ValueError):
        ObservationSampler(period=0)


def test_every_processed_image_has_footprint_even_between_coverage_publications():
    sampler = ObservationSampler()
    assert sampler.sample('a', 0, 0, 0, 0)
    assert sampler.sample('a', .1, 3, 4, .5) is None
    footprint = sampler.footprint(3, 4, .5)
    assert len(footprint) == 4
    assert footprint == ObservationSampler().sample('a', .1, 3, 4, .5)
    assert sampler.sample('a', .2, 3, 4, .5) is None
