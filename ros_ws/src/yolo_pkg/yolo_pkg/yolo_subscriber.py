import math
import sys
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from yolo_pkg.cow_tracker import CowTrackerManager


ENABLE_REALTIME_TRACK_PLOT = True
PLOT_HISTORY_MS = 10000
PLOT_POSITION_GRID_METERS = 1.0
PLOT_MAP_X_LIMITS = (-10.0, 10.0)
PLOT_MAP_Y_LIMITS = (-10.0, 10.0)


class RealtimeCowPlotter:
    def __init__(
        self,
        history_ms=PLOT_HISTORY_MS,
        grid_meters=PLOT_POSITION_GRID_METERS,
        map_x_limits=PLOT_MAP_X_LIMITS,
        map_y_limits=PLOT_MAP_Y_LIMITS,
    ):
        self.history_seconds = max(float(history_ms) / 1000.0, 0.1)
        self.grid_meters = max(float(grid_meters), 0.1)
        self.map_x_limits = tuple(float(value) for value in map_x_limits)
        self.map_y_limits = tuple(float(value) for value in map_y_limits)
        self.history = {}
        self.window_name = 'Cow tracker debug'
        self.canvas_width = 1100
        self.canvas_height = 900
        self.position_rect = (70, 55, 670, 655)
        self.noise_rect = (70, 735, 1065, 860)
        self._create_window()

    def update(self, timestamp, tracks, measurements):
        timestamp = float(timestamp)
        measurement_array = np.asarray(measurements, dtype=float).reshape(-1, 2)
        active_track_ids = set()

        for track in tracks:
            track_id = int(track['id'])
            active_track_ids.add(track_id)
            x = float(track['x'])
            y = float(track['y'])
            noise = self._nearest_measurement_distance(x, y, measurement_array)

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

    def _nearest_measurement_distance(self, x, y, measurements):
        if len(measurements) == 0:
            return np.nan

        deltas = measurements - np.array([[x, y]], dtype=float)
        distances = np.linalg.norm(deltas, axis=1)
        return float(np.min(distances))

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

    def _equal_xy_limits(self, x_values, y_values, rect):
        if not x_values or not y_values:
            return (-5.0, 5.0), (-5.0, 5.0)

        min_x = float(np.min(x_values))
        max_x = float(np.max(x_values))
        min_y = float(np.min(y_values))
        max_y = float(np.max(y_values))
        center_x = 0.5 * (min_x + max_x)
        center_y = 0.5 * (min_y + max_y)

        width_m = max(max_x - min_x, 2.0)
        height_m = max(max_y - min_y, 2.0)
        left, top, right, bottom = rect
        pixel_aspect = (right - left) / (bottom - top)

        if width_m / height_m > pixel_aspect:
            height_m = width_m / pixel_aspect
        else:
            width_m = height_m * pixel_aspect

        width_m = math.ceil(width_m / self.grid_meters) * self.grid_meters
        height_m = math.ceil(height_m / self.grid_meters) * self.grid_meters
        return (center_x - width_m / 2.0, center_x + width_m / 2.0), (center_y - height_m / 2.0, center_y + height_m / 2.0)

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


class YoloPersonSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pkg')

        self.namespace = sys.argv[1]
        number_of_drones = int(sys.argv[2])

        self.drone_image_sub_list = {}
        self.drone_pos_sub_list = {}
        self.drone_pos_dir = {}

        for i in range(number_of_drones):
            name = self.namespace + str(i)
            self.drone_image_sub_list[name] = self.create_subscription(
                Image,
                name + '/front/image_raw',
                lambda msg, n=name: self.listener_callback(msg, n),
                10,
            )
            self.drone_pos_sub_list[name] = self.create_subscription(
                Pose,
                name + '/gt_pose',
                lambda msg, n=name: self.drone_callback(msg, n),
                10,
            )

        self.publisher = self.create_publisher(PoseArray, '/cows_pos', 10)
        self.tracker = CowTrackerManager(
            dt=0.1,
            max_lost_frames=15,
            mahalanobis_gate=9.21,
            min_hits=2,
            max_dt=1.0,
        )
        self.plotter = RealtimeCowPlotter() if ENABLE_REALTIME_TRACK_PLOT else None

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

    def drone_callback(self, msg, namespace):
        self.drone_pos_dir[namespace] = msg

    def listener_callback(self, msg, name):
        if name not in self.drone_pos_dir:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        detections = results[0].boxes

        drone_pose = self.drone_pos_dir[name]
        drone_x = drone_pose.position.x
        drone_y = drone_pose.position.y
        drone_qz = drone_pose.orientation.z
        drone_qw = drone_pose.orientation.w
        drone_yaw = 2.0 * math.atan2(drone_qz, drone_qw)

        global_measurements = []

        for box in detections:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            if label not in ('cow', 'horse', 'giraffe'):
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            if y2 < 206:
                continue

            distance_1 = -(1.0 / 0.7714) * np.log((y2 - 205.31) / 363.34)
            center_x = ((x2 - x1) / 2.0) + x1
            image_angle = (1.0 / 0.01745) * np.log(
                (abs(center_x - 320.0) + 167.69) / 171.46
            )
            full_distance = abs(distance_1 / np.cos(image_angle * (np.pi / 180.0)))

            if not np.isfinite(full_distance):
                continue

            sign = 1.0 if center_x >= 320.0 else -1.0
            bearing_offset = sign * image_angle * (np.pi / 180.0)
            world_bearing = drone_yaw - bearing_offset
            cow_x = drone_x + math.cos(world_bearing) * full_distance
            cow_y = drone_y + math.sin(world_bearing) * full_distance
            global_measurements.append((cow_x, cow_y))

        timestamp = self._stamp_to_seconds(msg.header.stamp)
        tracks = self.tracker.update(global_measurements, timestamp=timestamp)
        if self.plotter is not None:
            self.plotter.update(timestamp, tracks, global_measurements)
        self._publish_tracks(tracks, msg.header)

    def _publish_tracks(self, tracks, header):
        tracked_cows = PoseArray()
        tracked_cows.header = header
        tracked_cows.header.frame_id = 'map'

        for track in tracks:
            pose = Pose()
            pose.position.x = float(track['x'])
            pose.position.y = float(track['y'])
            pose.position.z = float(track['id'])
            pose.orientation.x = float(track['vx'])
            pose.orientation.y = float(track['vy'])
            pose.orientation.z = 1.0 if track['coasting'] else 0.0
            pose.orientation.w = 1.0
            tracked_cows.poses.append(pose)

        self.publisher.publish(tracked_cows)

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
