"""Exercise the real camera callback with deterministic inference, no weights/GPU."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from yolo_pkg import yolo_subscriber
from yolo_pkg.camera_projection import project_box


class FakeModel:
    names = {0: 'cow', 1: 'person'}

    def __init__(self):
        self.boxes = []
        self.fail = False

    def __call__(self, *args, **kwargs):
        if self.fail:
            raise RuntimeError('inference unavailable')
        return [SimpleNamespace(boxes=self.boxes)]


def box(left=280, right=360, bottom=240, label=0):
    return SimpleNamespace(cls=[label], xyxy=[[left, 100, right, bottom]], conf=[.9])


def image(seconds=10):
    msg = Image()
    msg.header.stamp.sec = seconds
    return msg


def odometry(seconds, x, y, yaw=0):
    msg = Odometry()
    msg.header.stamp.sec = seconds
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw/2)
    msg.pose.pose.orientation.w = math.cos(yaw/2)
    return msg


@pytest.fixture
def camera(monkeypatch):
    model = FakeModel()
    monkeypatch.setattr(yolo_subscriber, 'YOLO', lambda _: model)
    rclpy.init(args=['--ros-args', '-p', 'drone_namespace_prefix:=/raw_drone',
                     '-p', 'number_of_drones:=2', '-p', 'cow_positions_topic:=/raw_cows'])
    node = yolo_subscriber.YoloCowSubscriber()
    observations, positions, coverage = [], [], []
    node.cow_observation_publisher = SimpleNamespace(publish=observations.append)
    node.publisher = SimpleNamespace(publish=positions.append)
    node.observation_publishers['/raw_drone0'] = SimpleNamespace(publish=coverage.append)
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.zeros((480, 640, 3), np.uint8))
    try:
        yield node, model, observations, positions, coverage
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_per_camera_raw_hit_empty_and_no_tracker(camera):
    node, model, observations, positions, coverage = camera
    node.drone_callback(odometry(9, 0, 0), '/raw_drone0')
    node.drone_callback(odometry(11, 2, 4), '/raw_drone0')
    model.boxes = [box(), box(label=1)]
    node.listener_callback(image(), '/raw_drone0')
    assert not hasattr(node, 'tracker')
    # Camera 1 supplied no frame: camera 0 must publish without batching.
    assert len(observations) == len(positions) == 1
    obs = observations[0]
    assert obs.header.frame_id == 'map' and obs.header.stamp.sec == 10
    assert obs.source_drone_index == 0
    assert obs.camera_pose.position.x == 1 and obs.camera_pose.position.y == 2
    assert obs.near_range_m == 2 and obs.far_range_m == 20
    assert len(obs.detections) == len(positions[0].poses) == 1
    detection = obs.detections[0]
    expected = project_box(node.config, 280, 360, 240, (1, 2, 0))
    assert detection.position_valid
    assert detection.position.x == pytest.approx(expected.x)
    assert detection.position.y == pytest.approx(expected.y)
    assert detection.bearing_min_rad < 0 < detection.bearing_max_rad
    assert coverage  # visited-area sampling continues independently
    model.boxes = []
    node.listener_callback(image(), '/raw_drone0')
    assert len(observations) == 2 and not observations[-1].detections
    assert not positions[-1].poses  # no remembered cow republished


def test_invalid_depth_protects_bearing_and_does_not_publish_position(camera):
    node, model, observations, positions, _ = camera
    node.drone_callback(odometry(10, 0, 0), '/raw_drone0')
    model.boxes = [box(bottom=190)]
    node.listener_callback(image(), '/raw_drone0')
    assert len(observations[0].detections) == 1
    assert not observations[0].detections[0].position_valid
    assert not positions[0].poses


def test_missing_pose_failed_image_and_failed_inference_are_not_empty(camera):
    node, model, observations, positions, _ = camera
    node.listener_callback(image(), '/raw_drone0')
    assert not observations
    node.drone_callback(odometry(10, 0, 0), '/raw_drone0')
    model.fail = True
    node.listener_callback(image(), '/raw_drone0')
    assert not observations and not positions
    model.fail = False
    node.bridge.imgmsg_to_cv2 = lambda *a, **k: (_ for _ in ()).throw(ValueError('bad image'))
    node.listener_callback(image(), '/raw_drone0')
    assert not observations and not positions


def test_source_index_pose_rotation_and_clock_reset(camera):
    node, model, observations, positions, _ = camera
    node.drone_callback(odometry(10, 3, 4, math.pi/2), '/raw_drone1')
    model.boxes = [box()]
    node.listener_callback(image(), '/raw_drone1')
    obs = observations[0]
    assert obs.source_drone_index == 1
    assert positions[0].poses[0].position.z == 1
    assert obs.detections[0].position.y > 4
    node.drone_callback(odometry(1, -3, -4), '/raw_drone1')
    assert len(node.drone_pose_history['/raw_drone1']) == 1
    node.listener_callback(image(1), '/raw_drone1')
    assert observations[-1].camera_pose.position.x == -3


def test_projection_matches_legacy_center_and_rotates_box_shadow(camera):
    node, _, _, _, _ = camera
    cfg = node.config
    for left, right in [(50, 150), (280, 360), (450, 550)]:
        center = (left + right)/2
        distance_forward = -math.log((240-cfg.distance_bottom_offset_px)
                                     / cfg.distance_log_denominator)/cfg.distance_log_scale
        angle = math.log((abs(center-cfg.image_center_x_px)+cfg.bearing_center_offset_px)
                         / cfg.bearing_log_denominator)/cfg.bearing_log_scale * math.pi/180
        distance = abs(distance_forward/math.cos(angle))
        bearing = -angle if center >= cfg.image_center_x_px else angle
        p = project_box(cfg, left, right, 240, (2, 3, .4))
        assert p.position_valid
        assert p.x == pytest.approx(2+math.cos(.4+bearing)*distance)
        assert p.y == pytest.approx(3+math.sin(.4+bearing)*distance)
        assert p.bearing_min <= bearing <= p.bearing_max
