import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
import math
import sys
import time

class DronePositionControl(Node):
    def __init__(self):
        super().__init__('drone_position_controller')

        # Subscribers
        namespace = sys.argv[1]

        self.drone_sub = self.create_subscription(
            Pose, namespace + '/gt_pose', self.drone_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, namespace + '/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, namespace + '/land', 10)

        self.goto_sub = self.create_subscription(Pose, namespace + '/goto', self.goto_sub, 10)
        self.focusin_sub = self.create_subscription(Pose, namespace + '/focusin', self.focusin_sub, 10)

        # Posicao do drone
        self.drone_x = None
        self.drone_y = None
        self.drone_angle = None

        self.linear_velocity = 0.4
        self.angular_velocity = 0.2
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 1.0

        self.final_position_x = None
        self.final_position_y = None
        self.camera_focus_position_x = 0.0
        self.camera_focus_position_y = 0.0
        self.tolerance = 0.4

        # Takeoff and wait
        print(f"Takeoff in drone_position_controll -- {namespace}")
        self.wait_for_subscribers(self.takeoff_publisher, namespace + '/takeoff')
        time.sleep(8)
        self.takeoff_publisher.publish(Empty())

        # Timer to control the drone's movement
        self.create_timer(0.1, self.move_to_position)

    def wait_for_subscribers(self, publisher, topic_name, timeout=10.0):
        start_time = time.time()
        while publisher.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscribers on {topic_name}...")
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"Timeout waiting for subscribers on {topic_name}")
                break


    def quaternion_to_yaw(self, orientation):
        q_w = orientation.w
        q_x = orientation.y
        q_y = orientation.y
        q_z = orientation.z
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def drone_callback(self, msg):
        self.drone_x = msg.position.x
        self.drone_y = msg.position.y
        self.drone_angle = self.quaternion_to_yaw(msg.orientation)

    def goto_sub(self, msg):
        self.final_position_x = msg.position.x
        self.final_position_y = msg.position.y

    def focusin_sub(self, msg):
        self.camera_focus_position_x = msg.position.x
        self.camera_focus_position_y = msg.position.y

    def move_to_position(self):
        if self.drone_x is None or self.drone_y is None or self.final_position_x is None or self.final_position_y is None:
            return

        # Calculate the distance to the final position
        distance_to_final = math.sqrt((self.final_position_x - self.drone_x)**2 +
                                      (self.final_position_y - self.drone_y)**2)

        # Calculate the distance to the camera focus position
        distance_to_focus = math.sqrt((self.camera_focus_position_x - self.drone_x)**2 +
                                      (self.camera_focus_position_y - self.drone_y)**2)

        if distance_to_final <= self.tolerance:
            # Stop translation, but still rotate to face camera focus if needed
            angular_vec = Vector3()
            if distance_to_focus > 0.5:
                angle_to_focus = math.atan2(self.camera_focus_position_y - self.drone_y,
                                            self.camera_focus_position_x - self.drone_x)
                angle_difference = angle_to_focus - self.drone_angle
                
                # Normalize the angle difference to the range [-pi, pi]
                angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

                if abs(angle_difference) > 0.02:
                    angular_vec.z = self.max_angular_velocity * (angle_difference / math.pi)
                else:
                    angular_vec.z = 0.0
            self.publish_cmd_vel(Vector3(), angular_vec)
            self.get_logger().info("Drone reached the destination. Spinning/focusing in place.")
        else:
            # Calculate the direction vector to the final position
            direction_x = self.final_position_x - self.drone_x
            direction_y = self.final_position_y - self.drone_y

            # Normalize the direction vector
            magnitude = math.sqrt(direction_x**2 + direction_y**2)
            direction_x /= magnitude
            direction_y /= magnitude

            # Set the linear velocity in the direction of the target
            linear_vec = Vector3()

            # Rotate the linear vector according to the drone's angle
            rotated_x = self.linear_velocity * (direction_x * math.cos(self.drone_angle) + direction_y * math.sin(self.drone_angle))
            rotated_y = self.linear_velocity * (-direction_x * math.sin(self.drone_angle) + direction_y * math.cos(self.drone_angle))

            linear_vec.x = rotated_x
            linear_vec.y = rotated_y

            # Calculate the angular velocity to face the camera focus position
            angular_vec = Vector3()
            if distance_to_focus > 0.5:
                angle_to_focus = math.atan2(self.camera_focus_position_y - self.drone_y,
                                            self.camera_focus_position_x - self.drone_x)
                angle_difference = angle_to_focus - self.drone_angle
                
                # Normalize the angle difference to the range [-pi, pi]
                angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

                if abs(angle_difference) > 0.02:
                    angular_vec.z = self.max_angular_velocity * (angle_difference / math.pi)
                else:
                    angular_vec.z = 0.0

                #linear_vec.x = linear_vec.x * math.cos(angle_difference)
                #linear_vec.y = linear_vec.y * math.sin(angle_difference)
                
                #print(f"Angle to Focus {angle_to_focus}")
                #print(f"Drone {self.drone_angle}")
                #print(f"Diff {angle_difference}")

            #print(f"Vec X -- {linear_vec.x}")
            #print(f"Vec Y -- {linear_vec.y}")

            #print(f"Pose X -- {self.drone_x}")
            #print(f"Pose Y -- {self.drone_y}")


            self.publish_cmd_vel(linear_vec, angular_vec)

    def publish_cmd_vel(self, linear_vec=Vector3(), angular_vec=Vector3()):
        twist = Twist(linear=linear_vec, angular=angular_vec)
        self.cmd_vel_publisher.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl()
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()