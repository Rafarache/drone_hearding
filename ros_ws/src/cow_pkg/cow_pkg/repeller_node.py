import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node

from cow_pkg.config import load_cow_config


class PeopleRepeller(Node):
    def __init__(self):
        super().__init__('cow_repeller')

        self.config = load_cow_config()
        self.drone_namespace_prefix = str(
            self.declare_parameter('drone_namespace_prefix', '').value
        )
        self.cow_name_prefix = str(
            self.declare_parameter('cow_name_prefix', '').value
        )
        self.number_of_drones = int(
            self.declare_parameter('number_of_drones', 0).value
        )
        self.number_of_cows = int(
            self.declare_parameter('number_of_cows', 0).value
        )
        self.drone_approach_radius = float(
            self.declare_parameter('drone_approach_radius', -1.0).value
        )
        self.push_point_margin = float(
            self.declare_parameter('push_point_margin', -1.0).value
        )
        self._validate_shared_parameters()

        self.drone_sub_list = {}
        self.drone_pos_dir = {}
        for index in range(self.number_of_drones):
            name = self.drone_namespace_prefix + str(index)
            self.drone_sub_list[name] = self.create_subscription(
                Pose,
                name + '/gt_pose',
                lambda msg, n=name: self.drone_callback(msg, n),
                10,
            )

        self.position_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.models_callback,
            10,
        )

        self.cows_dict = None
        self.last_cows_dict = None

        minimum_sensing_radius = (
            self.drone_approach_radius + self.push_point_margin
        )
        if self.config.drone_sensing_radius_m <= minimum_sensing_radius:
            raise ValueError(
                'cow_config.xml drone_sensing_radius_m must be greater than '
                'the shared drone approach radius plus push-point margin'
            )

        self.publishers_dict = {}
        for index in range(self.number_of_cows):
            cow_key = self.cow_name_prefix + str(index)
            self.publishers_dict[cow_key] = self.create_publisher(
                Twist,
                f'/{cow_key}/cmd_vel',
                10,
            )

        self.timer = self.create_timer(
            self.config.control_period_seconds,
            self.move_cows,
        )

        self.get_logger().info(
            f'Loaded cow configuration: {self.config.source_path}; '
            f'drones={self.number_of_drones}, cows={self.number_of_cows}, '
            f'drone sensing={self.config.drone_sensing_radius_m:.2f} m, '
            f'cow repulsion={self.config.cow_repulsion_radius_m:.2f} m, '
            f'approach={minimum_sensing_radius:.2f} m'
        )

    def _validate_shared_parameters(self):
        if not self.drone_namespace_prefix:
            raise ValueError('drone_namespace_prefix cannot be empty')
        if not self.cow_name_prefix:
            raise ValueError('cow_name_prefix cannot be empty')
        if self.number_of_drones < 1:
            raise ValueError('number_of_drones must be at least one')
        if self.number_of_cows < 1:
            raise ValueError('number_of_cows must be at least one')
        if (
            not math.isfinite(self.drone_approach_radius)
            or self.drone_approach_radius <= 0.0
        ):
            raise ValueError('drone_approach_radius must be positive')
        if (
            not math.isfinite(self.push_point_margin)
            or self.push_point_margin < 0.0
        ):
            raise ValueError('push_point_margin must be nonnegative')

    @staticmethod
    def quaternion_to_yaw(orientation):
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_rad_neg_pi_to_pi(angle_rad):
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def move_cows(self):
        if self.cows_dict is None or not self.publishers_dict:
            return

        for key, value in self.cows_dict.items():
            if value is None or key not in self.publishers_dict:
                continue

            px = value.position.x
            py = value.position.y
            force_x = 0.0
            force_y = 0.0

            for drone_pose in self.drone_pos_dir.values():
                if drone_pose is None:
                    continue
                delta_x = px - drone_pose.position.x
                delta_y = py - drone_pose.position.y
                distance = math.hypot(delta_x, delta_y)
                if (
                    0.0
                    < distance
                    <= self.config.drone_sensing_radius_m
                ):
                    magnitude = 1.0 / (distance * distance)
                    force_x += magnitude * delta_x / distance
                    force_y += magnitude * delta_y / distance

            for other_key, other_pose in self.cows_dict.items():
                if other_key == key or other_pose is None:
                    continue
                delta_x = px - other_pose.position.x
                delta_y = py - other_pose.position.y
                distance = math.hypot(delta_x, delta_y)
                if (
                    0.0
                    < distance
                    < self.config.cow_repulsion_radius_m
                ):
                    magnitude = 1.0 / (distance * distance)
                    force_x += magnitude * delta_x / distance
                    force_y += magnitude * delta_y / distance

            if math.hypot(force_x, force_y) == 0.0:
                self.publishers_dict[key].publish(Twist())
                continue

            target_yaw = math.atan2(force_y, force_x)
            yaw = self.quaternion_to_yaw(value.orientation)
            if (
                self.last_cows_dict is not None
                and key in self.last_cows_dict
                and self.last_cows_dict[key] is not None
            ):
                last_yaw = self.quaternion_to_yaw(
                    self.last_cows_dict[key].orientation
                )
            else:
                last_yaw = yaw

            yaw_error = self.normalize_rad_neg_pi_to_pi(target_yaw - yaw)
            last_yaw_error = self.normalize_rad_neg_pi_to_pi(
                target_yaw - last_yaw
            )

            twist = Twist()
            twist.linear.x = self.config.linear_velocity_mps
            twist.angular.z = (
                yaw_error * self.config.heading_proportional_gain
                + self.config.heading_derivative_gain
                * (yaw_error - last_yaw_error)
            )
            self.publishers_dict[key].publish(twist)

    def drone_callback(self, msg, namespace):
        self.drone_pos_dir[namespace] = msg

    def models_callback(self, msg):
        dictionary = {}
        self.last_cows_dict = self.cows_dict

        for index, name in enumerate(msg.name):
            if not name.startswith(self.cow_name_prefix):
                continue
            dictionary[name] = msg.pose[index]

        self.cows_dict = dictionary


def main(args=None):
    rclpy.init(args=args)
    node = PeopleRepeller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
