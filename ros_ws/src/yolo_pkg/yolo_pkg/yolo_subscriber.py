import math
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PolygonStamped, Point32
from cow_mapping_interfaces.msg import CowDetection, CowObservation
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from yolo_pkg.config import load_yolo_config
from yolo_pkg.search_observation import ObservationSampler
from yolo_pkg.camera_projection import project_box


class YoloCowSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pkg')

        self.config = load_yolo_config()
        self.namespace = str(
            self.declare_parameter('drone_namespace_prefix', '').value
        )
        number_of_drones = int(
            self.declare_parameter('number_of_drones', 0).value
        )
        self.cow_positions_topic = str(
            self.declare_parameter('cow_positions_topic', '').value
        )
        self.cow_observations_topic = str(
            self.declare_parameter('cow_observations_topic', '/cow_observations').value
        )
        if not self.cow_observations_topic:
            raise ValueError('cow_observations_topic cannot be empty')
        if not self.namespace:
            raise ValueError('drone_namespace_prefix cannot be empty')
        if number_of_drones < 1:
            raise ValueError('number_of_drones must be at least one')
        if not self.cow_positions_topic:
            raise ValueError('cow_positions_topic cannot be empty')

        near = float(self.declare_parameter('search_near_m', 2.0).value)
        far = float(self.declare_parameter('search_far_m', 10.0).value)
        fov = float(self.declare_parameter('search_fov_rad', self.config.horizontal_fov_rad).value)
        if far > self.config.max_detection_distance_m or fov > self.config.horizontal_fov_rad:
            raise ValueError('Search footprint exceeds configured detector visibility')
        observation_period = float(self.declare_parameter('search_observation_period_s', 1.0).value)
        self.observation_sampler = ObservationSampler(near, far, fov, observation_period)
        self.observation_publishers = {}
        self.drone_image_sub_list = {}
        self.drone_pos_sub_list = {}
        self.drone_pose_history = {}
        self.drone_index_by_name = {}
        self.show_annotated_camera = self.config.show_annotated_camera
        self.camera_horizontal_fov_rad = self.config.horizontal_fov_rad
        camera_view_state = (
            'enabled' if self.show_annotated_camera else 'disabled'
        )
        self.get_logger().info(
            f'Loaded YOLO configuration: {self.config.source_path}; '
            f'annotated camera view is {camera_view_state}'
        )

        for i in range(number_of_drones):
            name = self.namespace + str(i)
            self.observation_publishers[name] = self.create_publisher(
                PolygonStamped, name + '/processed_view', 50)
            self.drone_index_by_name[name] = i
            self.drone_pose_history[name] = deque(maxlen=self.config.pose_history_length)
            self.drone_image_sub_list[name] = self.create_subscription(
                Image,
                name + '/front/image_raw',
                lambda msg, n=name: self.listener_callback(msg, n),
                10,
            )
            self.drone_pos_sub_list[name] = self.create_subscription(
                Odometry,
                name + '/odom',
                lambda msg, n=name: self.drone_callback(msg, n),
                10,
            )

        self.publisher = self.create_publisher(PoseArray, self.cow_positions_topic, 10)
        self.cow_observation_publisher = self.create_publisher(
            CowObservation, self.cow_observations_topic, 50)
        self.get_logger().info('Cow tracking disabled; publishing raw camera observations')

        self.bridge = CvBridge()
        self.model = YOLO(self.config.model_path)

    def drone_callback(self, msg, namespace):
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        pose = msg.pose.pose
        history = self.drone_pose_history[namespace]
        if history and timestamp < history[-1][0]:
            history.clear()
        history.append(
            (
                timestamp,
                float(pose.position.x),
                float(pose.position.y),
                self._yaw_from_quaternion(pose.orientation),
            )
        )

    def listener_callback(self, msg, name):
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        drone_state = self._drone_state_at(name, timestamp)
        if drone_state is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(
                frame, conf=self.config.confidence_threshold, verbose=False)
            detections = results[0].boxes
        except Exception as exc:
            # A failed image is not evidence of empty space.
            self.get_logger().error(f'Camera processing failed: {exc}', throttle_duration_sec=5.0)
            return
        annotated_frame = (
            frame.copy() if self.show_annotated_camera else None
        )

        drone_x, drone_y, drone_yaw = drone_state

        observation = CowObservation()
        observation.header.stamp = msg.header.stamp
        observation.header.frame_id = 'map'
        observation.source_drone_index = self.drone_index_by_name[name]
        observation.camera_pose.position.x = float(drone_x)
        observation.camera_pose.position.y = float(drone_y)
        observation.camera_pose.orientation.z = math.sin(drone_yaw / 2.0)
        observation.camera_pose.orientation.w = math.cos(drone_yaw / 2.0)
        observation.near_range_m = 2.0
        observation.far_range_m = self.config.max_detection_distance_m
        observation.horizontal_fov_rad = self.camera_horizontal_fov_rad

        for box in detections:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            if label not in self.config.tracked_labels:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            projected = project_box(self.config, x1, x2, y2, drone_state)
            detection = CowDetection()
            detection.position_valid = projected.position_valid
            detection.position.x = projected.x
            detection.position.y = projected.y
            detection.bearing_min_rad = projected.bearing_min
            detection.bearing_max_rad = projected.bearing_max
            observation.detections.append(detection)
            full_distance = projected.distance

            if annotated_frame is not None:
                confidence = float(box.conf[0])
                cv2.rectangle(
                    annotated_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated_frame,
                    f'{label} {confidence:.2f} {full_distance:.2f}m',
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        self.cow_observation_publisher.publish(observation)
        self._publish_detections(observation)

        footprint = self.observation_sampler.sample(
            name, timestamp, drone_x, drone_y, drone_yaw)
        if footprint is not None:
            coverage_observation = PolygonStamped()
            coverage_observation.header.stamp = msg.header.stamp
            coverage_observation.header.frame_id = 'map'
            coverage_observation.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                                                   for x, y in footprint]
            self.observation_publishers[name].publish(coverage_observation)

        if annotated_frame is not None:
            cv2.imshow(f'{name} - YOLO cow detection', annotated_frame)
            cv2.waitKey(1)

    def _drone_state_at(self, name, timestamp):
        history = self.drone_pose_history.get(name)
        if not history:
            return None

        if timestamp <= history[0][0]:
            nearest = history[0]
            if nearest[0] - timestamp > self.config.max_pose_time_error_seconds:
                return None
            return nearest[1], nearest[2], nearest[3]

        if timestamp >= history[-1][0]:
            nearest = history[-1]
            if timestamp - nearest[0] > self.config.max_pose_time_error_seconds:
                return None
            return nearest[1], nearest[2], nearest[3]

        for newer_index in range(1, len(history)):
            newer = history[newer_index]
            if newer[0] < timestamp:
                continue

            older = history[newer_index - 1]
            interval = newer[0] - older[0]
            if interval <= 0.0:
                return newer[1], newer[2], newer[3]

            ratio = (timestamp - older[0]) / interval
            yaw_delta = (
                (newer[3] - older[3] + math.pi) % (2.0 * math.pi)
                - math.pi
            )
            x = older[1] + ratio * (newer[1] - older[1])
            y = older[2] + ratio * (newer[2] - older[2])
            yaw = older[3] + ratio * yaw_delta
            return x, y, yaw

        return None

    def _publish_detections(self, observation):
        raw_cows = PoseArray()
        raw_cows.header = observation.header
        for detection in observation.detections:
            if not detection.position_valid:
                continue
            pose = Pose()
            pose.position.x = detection.position.x
            pose.position.y = detection.position.y
            # Preserve the legacy raw PoseArray source-index convention.
            pose.position.z = float(observation.source_drone_index)
            pose.orientation.w = 1.0
            raw_cows.poses.append(pose)
        self.publisher.publish(raw_cows)

    @staticmethod
    def _yaw_from_quaternion(orientation):
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args=None):
    rclpy.init(args=args)
    node = YoloCowSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
