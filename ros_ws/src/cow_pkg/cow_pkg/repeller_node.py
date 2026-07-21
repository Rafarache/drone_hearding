import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import EntityState
import math
import sys
import numpy as np

class PeopleRepeller(Node):
    def __init__(self):
        super().__init__('cow_repeller')

        namespace = sys.argv[1]
        number_of_drones = int(sys.argv[2])
        number_of_cows = int(sys.argv[3])
 
        # Subscribers
        self.drone_sub_list = {}
        self.drone_pos_dir = {}

        for i in range(number_of_drones):
            name = namespace + str(i)
            self.drone_sub_list[name] = (self.create_subscription(
            Pose, name + '/gt_pose', lambda msg, n=name: self.drone_callback(msg, n), 10))

        self.position_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.models_callback, 10)

        # Posicao do drone
        self.cows_dict = None
        self.last_cows_dict = None

        self.threshold = 5.0  # distancia limite para repelir

        self.publishers_dict = {}
        for i in range(number_of_cows):
            cow_key = 'cow' + str(i)
            publisher = self.create_publisher(Twist, f'/{cow_key}/cmd_vel', 10)
            self.publishers_dict[cow_key] = publisher

        self.timer = self.create_timer(0.1, self.move_cows)

    def quaternion_to_yaw(self, orientation):
        q_w = orientation.w
        q_x = orientation.x
        q_y = orientation.y
        q_z = orientation.z
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def invert_rad(self, rad):
        if rad > 0:
            return rad - math.pi
        else:
            return rad + math.pi

    def normalize_rad_neg_pi_to_pi(self, angle_rad):
        pi = math.pi
        two_pi = 2 * pi
        normalized = (angle_rad + pi) % two_pi - pi
        return normalized

    def move_cows(self):
        if self.cows_dict is None or len(self.publishers_dict.keys()) == 0:
            return

        drone_threshold = 5.0   # metres - repulsion radius from drones
        cow_threshold   = 1.0   # metres - repulsion radius from other cows

        for key, value in self.cows_dict.items():
            if value is None:
                continue
            if key not in self.publishers_dict:
                continue

            px = value.position.x
            py = value.position.y

            # ── Accumulate APF repulsive forces ──────────────────────────────
            fx = 0.0
            fy = 0.0

            # Repulsion from drones
            for drone_key, drone_pose in self.drone_pos_dir.items():
                if drone_pose is None:
                    continue
                dx = px - drone_pose.position.x
                dy = py - drone_pose.position.y
                dist = math.sqrt(dx * dx + dy * dy)
                if 0.0 < dist < drone_threshold:
                    # APF: force magnitude = 1/d²  (points away from drone)
                    magnitude = 1.0 / (dist * dist)
                    fx += magnitude * (dx / dist)
                    fy += magnitude * (dy / dist)

            # Repulsion from other cows
            for other_key, other_pose in self.cows_dict.items():
                if other_key == key or other_pose is None:
                    continue
                dx = px - other_pose.position.x
                dy = py - other_pose.position.y
                dist = math.sqrt(dx * dx + dy * dy)
                if 0.0 < dist < cow_threshold:
                    magnitude = 1.0 / (dist * dist)
                    fx += magnitude * (dx / dist)
                    fy += magnitude * (dy / dist)

            # If no force acts on this cow, publish zero and move on
            force_mag = math.sqrt(fx * fx + fy * fy)
            if force_mag == 0.0:
                self.publishers_dict[key].publish(Twist())
                continue

            # ── Steer toward the resultant force direction ────────────────────
            rad = math.atan2(fy, fx)
            yaw = self.quaternion_to_yaw(value.orientation)

            if self.last_cows_dict is not None and key in self.last_cows_dict and self.last_cows_dict[key] is not None:
                last_yaw = self.quaternion_to_yaw(self.last_cows_dict[key].orientation)
            else:
                last_yaw = yaw

            diff_rad      = self.normalize_rad_neg_pi_to_pi(rad - yaw)
            last_diff_rad = self.normalize_rad_neg_pi_to_pi(rad - last_yaw)

            control_p = 0.2
            control_d = 5.0

            twist = Twist()
            twist.linear.x  = 0.5
            twist.angular.z = (diff_rad * control_p) + (control_d * (diff_rad - last_diff_rad))

            self.publishers_dict[key].publish(twist)

    def drone_callback(self, msg, namespace):
        self.drone_pos_dir[namespace] = msg

    def models_callback(self, msg: ModelStates):
        dictionary = {}
        self.last_cows_dict = self.cows_dict

        for index, name in enumerate(msg.name):

            if 'cow' not in name.lower():
                continue

            pose = msg.pose[index]
            px = pose.position.x
            py = pose.position.y
            dictionary[name] = pose

        self.cows_dict = dictionary

            
def main(args=None):
    rclpy.init(args=args)
    node = PeopleRepeller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
