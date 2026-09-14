import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from yolo_pkg.config import load_yolo_config
from yolo_pkg.cow_tracker import CowTrackerManager


class RealtimeCowPlotter:
    def __init__(self, config):
        self.history_seconds = max(
            float(config.plot_history_ms) / 1000.0, 0.1
        )
        self.grid_meters = max(float(config.plot_grid_m), 0.1)
        self.map_x_limits = (config.plot_min_x, config.plot_max_x)
        self.map_y_limits = (config.plot_min_y, config.plot_max_y)
        self.history = {}
        self.window_name = 'Cow tracker debug'
        self.canvas_width = 1100
        self.canvas_height = 900
        self.position_rect = (70, 55, 670, 655)
        self.noise_rect = (70, 735, 1065, 860)
        self._create_window()

    def update(self, timestamp, tracks):
        timestamp = float(timestamp)
        active_track_ids = set()

        for track in tracks:
            track_id = int(track['id'])
            active_track_ids.add(track_id)
            x = float(track['x'])
            y = float(track['y'])
            noise = float(track.get('innovation_norm', np.nan))

            values = self.history.setdefault(track_id, [])
            values.append(
                {
                    'time': timestamp,
                    'x': x,
                    'y': y,
                    'noise': noise,
                    'coasting': bool(track['coasting']),
                }
            )

        self._trim_history(timestamp, active_track_ids)
        self._draw(timestamp)

    def _create_window(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.canvas_width, self.canvas_height)

    def _trim_history(self, timestamp, active_track_ids):
        min_time = timestamp - self.history_seconds
        for track_id in list(self.history):
            self.history[track_id] = [
                sample for sample in self.history[track_id]
                if sample['time'] >= min_time
            ]
            if not self.history[track_id] and track_id not in active_track_ids:
                del self.history[track_id]


    def _draw(self, timestamp):
        if not self._window_is_available():
            self._create_window()

        canvas = np.full((self.canvas_height, self.canvas_width, 3), 255, dtype=np.uint8)
        self._draw_axes(
            canvas,
            self.position_rect,
            'Filtered cow positions - fixed -10 to +10 map',
            'X [m]',
            'Y [m]',
        )
        self._draw_axes(
            canvas,
            self.noise_rect,
            'Detection noise',
            'time [s]',
            'm',
        )

        noise_values = []
        for samples in self.history.values():
            noise_values.extend(
                sample['noise'] for sample in samples
                if np.isfinite(sample['noise'])
            )

        x_limits = self.map_x_limits
        y_limits = self.map_y_limits
        noise_limits = self._limits(noise_values, default=(0.0, 1.0), pad=0.1)
        time_limits = (max(0.0, timestamp - self.history_seconds), timestamp)
        if math.isclose(time_limits[0], time_limits[1]):
            time_limits = (time_limits[0], time_limits[0] + 1.0)

        self._draw_meter_grid(canvas, self.position_rect, x_limits, y_limits)
        self._draw_time_grid(canvas, self.noise_rect, time_limits, noise_limits)

        for legend_index, track_id in enumerate(sorted(self.history)):
            color = self._color_for_track(track_id)
            samples = self.history[track_id]
            self._draw_position_trace(canvas, samples, x_limits, y_limits, color, timestamp)
            self._draw_noise_trace(canvas, samples, time_limits, noise_limits, color, timestamp)
            self._draw_legend_item(canvas, legend_index, track_id, color, samples)

        cv2.imshow(self.window_name, canvas)
        cv2.waitKey(1)

    def _window_is_available(self):
        try:
            return cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 1
        except cv2.error:
            return False

    def _draw_axes(self, canvas, rect, title, x_label, y_label):
        left, top, right, bottom = rect
        cv2.rectangle(canvas, (left, top), (right, bottom), (40, 40, 40), 1)
        cv2.putText(canvas, title, (left, top - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.putText(canvas, x_label, (right - 65, bottom + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(canvas, y_label, (left - 50, top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)

    def _draw_meter_grid(self, canvas, rect, x_limits, y_limits):
        left, top, right, bottom = rect
        min_x, max_x = x_limits
        min_y, max_y = y_limits
        first_x = math.floor(min_x / self.grid_meters) * self.grid_meters
        last_x = math.ceil(max_x / self.grid_meters) * self.grid_meters
        first_y = math.floor(min_y / self.grid_meters) * self.grid_meters
        last_y = math.ceil(max_y / self.grid_meters) * self.grid_meters

        x = first_x
        while x <= last_x:
            px, _ = self._scale_point(x, min_y, rect, x_limits, y_limits)
            cv2.line(canvas, (px, top), (px, bottom), (225, 225, 225), 1)
            cv2.putText(canvas, str(int(round(x))), (px + 2, bottom + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1, cv2.LINE_AA)
            x += self.grid_meters

        y = first_y
        while y <= last_y:
            _, py = self._scale_point(min_x, y, rect, x_limits, y_limits)
            cv2.line(canvas, (left, py), (right, py), (225, 225, 225), 1)
            cv2.putText(canvas, str(int(round(y))), (left - 35, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1, cv2.LINE_AA)
            y += self.grid_meters

    def _draw_time_grid(self, canvas, rect, time_limits, noise_limits):
        left, top, right, bottom = rect
        min_time, max_time = time_limits
        min_noise, max_noise = noise_limits

        for i in range(6):
            t = min_time + (max_time - min_time) * i / 5.0
            px, _ = self._scale_point(t, min_noise, rect, time_limits, noise_limits)
            cv2.line(canvas, (px, top), (px, bottom), (235, 235, 235), 1)

        for i in range(6):
            noise = min_noise + (max_noise - min_noise) * i / 5.0
            _, py = self._scale_point(min_time, noise, rect, time_limits, noise_limits)
            cv2.line(canvas, (left, py), (right, py), (235, 235, 235), 1)
            cv2.putText(canvas, '%.1f' % noise, (left - 45, py + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1, cv2.LINE_AA)

    def _draw_position_trace(self, canvas, samples, x_limits, y_limits, color, timestamp):
        if len(samples) < 2:
            return

        points = [
            self._scale_point(sample['x'], sample['y'], self.position_rect, x_limits, y_limits)
            for sample in samples
        ]
        ages = [timestamp - sample['time'] for sample in samples]
        self._draw_faded_polyline(canvas, points, ages, color)

    def _draw_noise_trace(self, canvas, samples, time_limits, noise_limits, color, timestamp):
        points = []
        ages = []
        for sample in samples:
            noise = sample['noise']
            if not np.isfinite(noise):
                continue
            points.append(
                self._scale_point(sample['time'], noise, self.noise_rect, time_limits, noise_limits)
            )
            ages.append(timestamp - sample['time'])
        self._draw_faded_polyline(canvas, points, ages, color)

    def _draw_faded_polyline(self, canvas, points, ages, color):
        if len(points) < 2:
            return

        for start, end, age in zip(points[:-1], points[1:], ages[1:]):
            alpha = self._opacity_for_age(age)
            if alpha <= 0.0:
                continue
            faded_color = self._blend_with_background(color, alpha)
            cv2.line(canvas, start, end, faded_color, 2, cv2.LINE_AA)

        latest_color = self._blend_with_background(color, self._opacity_for_age(ages[-1]))
        cv2.circle(canvas, points[-1], 4, latest_color, -1, cv2.LINE_AA)

    def _opacity_for_age(self, age):
        return max(0.0, min(1.0, 1.0 - age / self.history_seconds))

    def _blend_with_background(self, color, alpha):
        background = 255.0
        return tuple(
            int(round(background * (1.0 - alpha) + channel * alpha))
            for channel in color
        )

    def _draw_legend_item(self, canvas, legend_index, track_id, color, samples):
        y = 80 + 22 * legend_index
        x = 710
        label = 'cow %d' % track_id
        if samples and samples[-1]['coasting']:
            label += ' coast'
        cv2.line(canvas, (x, y), (x + 28, y), color, 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (x + 38, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)

    def _scale_point(self, x, y, rect, x_limits, y_limits):
        left, top, right, bottom = rect
        min_x, max_x = x_limits
        min_y, max_y = y_limits
        px = left + int((x - min_x) / (max_x - min_x) * (right - left))
        py = bottom - int((y - min_y) / (max_y - min_y) * (bottom - top))
        return px, py


    def _limits(self, values, default=(-1.0, 1.0), pad=0.1):
        if not values:
            return default

        min_value = float(np.min(values))
        max_value = float(np.max(values))
        if math.isclose(min_value, max_value):
            return min_value - 1.0, max_value + 1.0

        value_range = max_value - min_value
        return min_value - pad * value_range, max_value + pad * value_range

    def _color_for_track(self, track_id):
        hue = (track_id * 47) % 180
        hsv = np.uint8([[[hue, 190, 230]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])


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
        if not self.namespace:
            raise ValueError('drone_namespace_prefix cannot be empty')
        if number_of_drones < 1:
            raise ValueError('number_of_drones must be at least one')
        if not self.cow_positions_topic:
            raise ValueError('cow_positions_topic cannot be empty')

        self.drone_image_sub_list = {}
        self.drone_pos_sub_list = {}
        self.drone_pose_history = {}
        self.drone_index_by_name = {}
        self.expected_camera_names = set()
        self.pending_detection_batches = {}
        self.pending_batch_start_timestamp = None
        self.show_annotated_camera = self.config.show_annotated_camera
        self.camera_horizontal_fov_rad = self.config.horizontal_fov_rad
        self.tracker_reacquisition_distance = (
            self.config.tracker_reacquisition_distance_m
        )
        self.tracker_reacquisition_mahalanobis_gate = (
            self.config.tracker_reacquisition_mahalanobis_gate
        )
        camera_view_state = (
            'enabled' if self.show_annotated_camera else 'disabled'
        )
        self.get_logger().info(
            f'Loaded YOLO configuration: {self.config.source_path}; '
            f'annotated camera view is {camera_view_state}'
        )

        for i in range(number_of_drones):
            name = self.namespace + str(i)
            self.drone_index_by_name[name] = i
            self.expected_camera_names.add(name)
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
        self.tracker = CowTrackerManager(
            dt=self.config.tracker_default_dt_seconds,
            max_lost_frames=self.config.tracker_max_lost_frames,
            mahalanobis_gate=self.config.tracker_mahalanobis_gate,
            max_position_distance=self.config.tracker_max_position_distance_m,
            min_hits=self.config.tracker_min_confirmation_hits,
            max_dt=self.config.tracker_max_prediction_step_seconds,
            acceleration_noise_std=self.config.tracker_acceleration_noise_std,
            measurement_noise_std=self.config.tracker_measurement_noise_std,
            initial_velocity_std=self.config.tracker_initial_velocity_std,
            reacquisition_mahalanobis_gate=(
                self.tracker_reacquisition_mahalanobis_gate
            ),
            reacquisition_distance=self.tracker_reacquisition_distance,
        )
        self.last_tracker_diagnostic_log_time = None
        self.plotter = (
            RealtimeCowPlotter(self.config)
            if self.config.show_realtime_track_plot
            else None
        )

        self.bridge = CvBridge()
        self.model = YOLO(self.config.model_path)

    def drone_callback(self, msg, namespace):
        timestamp = self._stamp_to_seconds(msg.header.stamp)
        pose = msg.pose.pose
        self.drone_pose_history[namespace].append(
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

        self._flush_stale_detection_batch(timestamp)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(
            frame,
            conf=self.config.confidence_threshold,
            verbose=False,
        )
        detections = results[0].boxes
        annotated_frame = (
            frame.copy() if self.show_annotated_camera else None
        )

        drone_x, drone_y, drone_yaw = drone_state

        global_measurements = []

        for box in detections:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            if label not in self.config.tracked_labels:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            if y2 < self.config.minimum_box_bottom_px:
                continue

            distance_1 = -(1.0 / self.config.distance_log_scale) * np.log(
                (y2 - self.config.distance_bottom_offset_px)
                / self.config.distance_log_denominator
            )
            center_x = ((x2 - x1) / 2.0) + x1
            image_angle = (
                1.0 / self.config.bearing_log_scale
            ) * np.log(
                (
                    abs(center_x - self.config.image_center_x_px)
                    + self.config.bearing_center_offset_px
                )
                / self.config.bearing_log_denominator
            )
            full_distance = abs(distance_1 / np.cos(image_angle * (np.pi / 180.0)))

            if (
                not np.isfinite(full_distance)
                or full_distance <= 0.0
                or full_distance > self.config.max_detection_distance_m
            ):
                continue

            sign = (
                1.0
                if center_x >= self.config.image_center_x_px
                else -1.0
            )
            bearing_offset = sign * image_angle * (np.pi / 180.0)
            world_bearing = drone_yaw - bearing_offset
            cow_x = drone_x + math.cos(world_bearing) * full_distance
            cow_y = drone_y + math.sin(world_bearing) * full_distance
            global_measurements.append((cow_x, cow_y))

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

        if annotated_frame is not None:
            cv2.imshow(f'{name} - YOLO cow detection', annotated_frame)
            cv2.waitKey(1)

        if self.pending_batch_start_timestamp is None:
            self.pending_batch_start_timestamp = timestamp
        self.pending_detection_batches[name] = (
            timestamp,
            global_measurements,
            msg.header,
        )
        if self.expected_camera_names.issubset(
            self.pending_detection_batches
        ):
            self._process_detection_batch()

    def _flush_stale_detection_batch(self, timestamp):
        if self.pending_batch_start_timestamp is None:
            return
        if (
            timestamp - self.pending_batch_start_timestamp
            >= self.config.max_camera_batch_wait_seconds
        ):
            self._process_detection_batch()

    def _process_detection_batch(self):
        if not self.pending_detection_batches:
            return

        batches = self.pending_detection_batches
        self.pending_detection_batches = {}
        self.pending_batch_start_timestamp = None
        timestamp, _, header = max(
            batches.values(),
            key=lambda batch: batch[0],
        )
        measurements = self._merge_camera_measurements(batches)
        camera_views = []
        for source_name, (batch_timestamp, _, _) in batches.items():
            drone_state = self._drone_state_at(
                source_name, batch_timestamp
            )
            if drone_state is None:
                continue
            drone_x, drone_y, drone_yaw = drone_state
            camera_views.append(
                {
                    'x': drone_x,
                    'y': drone_y,
                    'yaw': drone_yaw,
                    'horizontal_fov': self.camera_horizontal_fov_rad,
                    'max_range': self.config.max_detection_distance_m,
                }
            )

        tracks = self.tracker.update(
            measurements,
            timestamp=timestamp,
            camera_views=camera_views,
        )
        self._log_tracker_diagnostics(timestamp)
        if self.plotter is not None:
            self.plotter.update(timestamp, tracks)

        source_name = min(
            batches,
            key=lambda camera_name: self.drone_index_by_name[camera_name],
        )
        source_drone_index = self.drone_index_by_name[source_name]
        self._publish_tracks(tracks, header, source_drone_index)

    def _log_tracker_diagnostics(self, timestamp):
        last_log_time = self.last_tracker_diagnostic_log_time
        if (
            last_log_time is not None
            and timestamp >= last_log_time
            and timestamp - last_log_time
            < self.config.tracker_diagnostic_log_period_seconds
        ):
            return

        self.last_tracker_diagnostic_log_time = timestamp
        diagnostics = self.tracker.last_diagnostics
        self.get_logger().info(
            'Cow tracker: '
            f"measurements={diagnostics['measurements']}, "
            f"tracks={diagnostics['total_tracks']}, "
            f"confirmed={diagnostics['confirmed_tracks']}, "
            f"coasting={diagnostics['coasting_tracks']}, "
            f"visible_misses={diagnostics['visible_misses']}, "
            f"reacquisitions={diagnostics['reacquisitions']}, "
            f"removed={diagnostics['removed_tracks']}"
        )

    def _merge_camera_measurements(self, batches):
        clusters = []
        for source_name, (_, measurements, _) in batches.items():
            for measurement in measurements:
                measurement = np.asarray(measurement, dtype=float)
                candidates = [
                    (np.linalg.norm(measurement - cluster['center']), cluster)
                    for cluster in clusters
                    if source_name not in cluster['sources']
                ]
                candidates = [
                    item
                    for item in candidates
                    if item[0] <= self.config.multi_camera_merge_distance_m
                ]

                if candidates:
                    _, cluster = min(candidates, key=lambda item: item[0])
                    cluster['sum'] += measurement
                    cluster['count'] += 1
                    cluster['sources'].add(source_name)
                    cluster['center'] = cluster['sum'] / cluster['count']
                else:
                    clusters.append(
                        {
                            'sum': measurement.copy(),
                            'count': 1,
                            'center': measurement.copy(),
                            'sources': {source_name},
                        }
                    )

        return [tuple(cluster['center']) for cluster in clusters]

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

    def _publish_tracks(self, tracks, header, source_drone_index):
        tracked_cows = PoseArray()
        tracked_cows.header.stamp = header.stamp
        tracked_cows.header.frame_id = 'map'

        for track in tracks:
            pose = Pose()
            pose.position.x = float(track['x'])
            pose.position.y = float(track['y'])
            # Keep the legacy PoseArray contract: z identifies the source drone.
            # Track IDs, velocities and coast state remain available to the plotter.
            pose.position.z = float(source_drone_index)
            pose.orientation.w = 1.0
            tracked_cows.poses.append(pose)

        self.publisher.publish(tracked_cows)

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
