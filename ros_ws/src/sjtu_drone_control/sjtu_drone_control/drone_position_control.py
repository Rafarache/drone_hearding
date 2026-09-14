import math
import time

import rclpy
from geometry_msgs.msg import Pose, Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Empty

from sjtu_drone_control.config import load_drone_control_config


class DronePositionControl(Node):
    def __init__(self):
        super().__init__('drone_position_controller')

        self.config = load_drone_control_config()
        namespace = str(
            self.declare_parameter('drone_namespace', '').value
        )
        if not namespace:
            raise ValueError('drone_namespace cannot be empty')

        self.drone_sub = self.create_subscription(
            Pose, namespace + '/gt_pose', self.drone_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, namespace + '/cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(
            Empty, namespace + '/takeoff', 10)
        self.land_publisher = self.create_publisher(
            Empty, namespace + '/land', 10)
        self.goto_subscriber = self.create_subscription(
            Pose, namespace + '/goto', self.goto_callback, 10)
        self.focusin_subscriber = self.create_subscription(
            Pose, namespace + '/focusin', self.focusin_callback, 10)

        self.max_linear_velocity = self.config.max_linear_velocity_mps
        self.max_linear_acceleration = (
            self.config.max_linear_acceleration_mps2
        )
        self.max_angular_velocity = self.config.max_angular_velocity_radps
        self.max_angular_acceleration = (
            self.config.max_angular_acceleration_radps2
        )
        self.position_tolerance = self.config.position_tolerance_m
        self.linear_slowdown_distance = (
            self.config.linear_slowdown_distance_m
        )
        self._validate_parameters()

        self.drone_x = None
        self.drone_y = None
        self.drone_angle = None
        self.final_position_x = None
        self.final_position_y = None
        self.camera_focus_position_x = 0.0
        self.camera_focus_position_y = 0.0

        # Linear velocity is retained in the global frame. Filtering before
        # the body-frame rotation makes a changing yaw independent of the
        # requested path direction.
        self.command_world_velocity_x = 0.0
        self.command_world_velocity_y = 0.0
        self.command_angular_velocity = 0.0
        self.last_control_time = None

        self.get_logger().info(
            f'Loaded drone control configuration: {self.config.source_path}; '
            'motion smoothing: '
            f'linear={self.max_linear_velocity:.2f} m/s, '
            f'linear_accel={self.max_linear_acceleration:.2f} m/s^2, '
            f'angular={self.max_angular_velocity:.2f} rad/s, '
            f'angular_accel={self.max_angular_acceleration:.2f} rad/s^2, '
            f'tolerance={self.position_tolerance:.2f} m, '
            f'slowdown={self.linear_slowdown_distance:.2f} m'
        )

        print(f'Takeoff in drone_position_controll -- {namespace}')
        self.wait_for_subscribers(
            self.takeoff_publisher, namespace + '/takeoff')
        time.sleep(self.config.takeoff_delay_seconds)
        self.takeoff_publisher.publish(Empty())

        self.last_control_time = time.monotonic()
        self.control_timer = self.create_timer(
            self.config.control_period_seconds, self.move_to_position)

    def _validate_parameters(self):
        positive_parameters = {
            'drone_max_linear_velocity': self.max_linear_velocity,
            'drone_max_linear_acceleration': self.max_linear_acceleration,
            'drone_max_angular_velocity': self.max_angular_velocity,
            'drone_max_angular_acceleration': self.max_angular_acceleration,
            'drone_linear_slowdown_distance': self.linear_slowdown_distance,
        }
        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')

        if (
            not math.isfinite(self.position_tolerance)
            or self.position_tolerance < 0.0
        ):
            raise ValueError(
                'drone_position_tolerance must be finite and nonnegative'
            )

    def wait_for_subscribers(self, publisher, topic_name):
        start_time = time.time()
        while publisher.get_subscription_count() == 0:
            self.get_logger().info(
                f'Waiting for subscribers on {topic_name}...')
            time.sleep(self.config.subscriber_poll_period_seconds)
            if (
                time.time() - start_time
                > self.config.subscriber_wait_timeout_seconds
            ):
                self.get_logger().warn(
                    f'Timeout waiting for subscribers on {topic_name}')
                break

    @staticmethod
    def quaternion_to_yaw(orientation):
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def drone_callback(self, msg):
        self.drone_x = float(msg.position.x)
        self.drone_y = float(msg.position.y)
        self.drone_angle = self.quaternion_to_yaw(msg.orientation)

    def goto_callback(self, msg):
        self.final_position_x = float(msg.position.x)
        self.final_position_y = float(msg.position.y)

    def focusin_callback(self, msg):
        self.camera_focus_position_x = float(msg.position.x)
        self.camera_focus_position_y = float(msg.position.y)

    def _control_dt(self):
        now = time.monotonic()
        if self.last_control_time is None:
            self.last_control_time = now
            return self.config.control_period_seconds

        elapsed = now - self.last_control_time
        self.last_control_time = now
        return min(max(elapsed, 0.0), self.config.max_control_dt_seconds)

    @staticmethod
    def _slew_vector(
        current_x,
        current_y,
        desired_x,
        desired_y,
        max_change,
    ):
        delta_x = desired_x - current_x
        delta_y = desired_y - current_y
        delta_magnitude = math.hypot(delta_x, delta_y)
        if delta_magnitude <= max_change or delta_magnitude == 0.0:
            return desired_x, desired_y

        scale = max_change / delta_magnitude
        return (
            current_x + delta_x * scale,
            current_y + delta_y * scale,
        )

    @staticmethod
    def _slew_scalar(current, desired, max_change):
        return current + max(-max_change, min(max_change, desired - current))

    def _desired_world_velocity(self):
        error_x = self.final_position_x - self.drone_x
        error_y = self.final_position_y - self.drone_y
        distance = math.hypot(error_x, error_y)
        distance_after_tolerance = max(
            0.0, distance - self.position_tolerance)
        if distance_after_tolerance == 0.0 or distance == 0.0:
            return 0.0, 0.0

        speed_scale = min(
            1.0,
            distance_after_tolerance / self.linear_slowdown_distance,
        )
        desired_speed = self.max_linear_velocity * speed_scale
        return (
            desired_speed * error_x / distance,
            desired_speed * error_y / distance,
        )

    def _desired_angular_velocity(self):
        focus_x = self.camera_focus_position_x - self.drone_x
        focus_y = self.camera_focus_position_y - self.drone_y
        if (
            math.hypot(focus_x, focus_y)
            <= self.config.focus_distance_tolerance_m
        ):
            return 0.0

        angle_to_focus = math.atan2(focus_y, focus_x)
        angle_error = (
            angle_to_focus - self.drone_angle + math.pi
        ) % (2.0 * math.pi) - math.pi
        if abs(angle_error) <= self.config.focus_angle_tolerance_rad:
            return 0.0

        return self.max_angular_velocity * angle_error / math.pi

    def move_to_position(self):
        if (
            self.drone_x is None
            or self.drone_y is None
            or self.drone_angle is None
            or self.final_position_x is None
            or self.final_position_y is None
        ):
            return

        dt = self._control_dt()
        desired_velocity_x, desired_velocity_y = (
            self._desired_world_velocity()
        )
        maximum_linear_change = self.max_linear_acceleration * dt
        (
            self.command_world_velocity_x,
            self.command_world_velocity_y,
        ) = self._slew_vector(
            self.command_world_velocity_x,
            self.command_world_velocity_y,
            desired_velocity_x,
            desired_velocity_y,
            maximum_linear_change,
        )

        desired_angular_velocity = self._desired_angular_velocity()
        self.command_angular_velocity = self._slew_scalar(
            self.command_angular_velocity,
            desired_angular_velocity,
            self.max_angular_acceleration * dt,
        )

        cosine = math.cos(self.drone_angle)
        sine = math.sin(self.drone_angle)
        linear_vector = Vector3()
        linear_vector.x = (
            self.command_world_velocity_x * cosine
            + self.command_world_velocity_y * sine
        )
        linear_vector.y = (
            -self.command_world_velocity_x * sine
            + self.command_world_velocity_y * cosine
        )

        angular_vector = Vector3()
        angular_vector.z = self.command_angular_velocity
        self.publish_cmd_vel(linear_vector, angular_vector)

    def publish_cmd_vel(self, linear_vector=None, angular_vector=None):
        if linear_vector is None:
            linear_vector = Vector3()
        if angular_vector is None:
            angular_vector = Vector3()
        self.cmd_vel_publisher.publish(
            Twist(linear=linear_vector, angular=angular_vector))


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
