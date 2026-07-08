import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics import solutions
import numpy as np
import math
import sys

class YoloPersonSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pkg')

        self.namespace = sys.argv[1]
        number_of_drones = int(sys.argv[2])

        # Subscribers
        self.drone_image_sub_list = {}
        self.drone_pos_sub_list = {}
        self.drone_pos_dir = {}

        for i in range(number_of_drones):
            name = self.namespace + str(i)
            self.drone_image_sub_list[name] = (self.create_subscription(
            Image, name + '/front/image_raw', lambda msg, n=name: self.listener_callback(msg, n), 10))
            self.drone_pos_sub_list[name] = (self.create_subscription(
            Pose, name + '/gt_pose', lambda msg, n=name: self.drone_callback(msg, n), 10))

        self.publisher = self.create_publisher(PoseArray, '/cows_pos', 10)

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # or 'yolov8s.pt' for better accuracy

    def drone_callback(self, msg, namespace):
        self.drone_pos_dir[namespace] = msg

    def listener_callback(self, msg, name):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(frame)

        # Visualize results
        detections = results[0].boxes
        annotated = frame.copy()

        if(name in self.drone_pos_dir):
            drone_x = self.drone_pos_dir[name].position.x
            drone_y = self.drone_pos_dir[name].position.y
            drone_qz = self.drone_pos_dir[name].orientation.z
            drone_qw = self.drone_pos_dir[name].orientation.w
            drone_yaw = 2 * math.atan2(drone_qz, drone_qw)  # radians

        # --- Para cada detecção ---
        detections_pos = PoseArray()

        for box in detections:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            if label != "cow" and label!='horse':
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if(y2 < 206):
                continue
            
            distance_1 = -(1/0.7714) * np.log((y2 - 205.31) / 363.34)
            distance_2 = -(1/0.7858) * np.log(((y2 - y1) - 54.23) / 720.50)
            center_x = ((x2 - x1) / 2 ) + x1
            angle = (1 / 0.01745) * np.log((abs(center_x - 320) + 167.69) / 171.46)
            full_distance = abs(distance_1 / np.cos(angle * (np.pi / 180)))

            if math.isnan(full_distance):
                continue

            # print(distance_1, distance_2, angle, full_distance)
            # Desenha box + texto
            # cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(annotated, f"{full_distance:.2}",
            #            (x1, y1 + 30), cv2.FONT_HERSHEY_SIMPLEX,
            #            0.7, (0, 255, 0), 2)

            if(name in self.drone_pos_dir):
                num = float(name[len(self.namespace):])
                sign = 1 if center_x >= 320 else -1
                angle = sign * (1 / 0.01745) * np.log((abs(center_x - 320) + 167.69) / 171.46)
                world_bearing = drone_yaw - angle * (np.pi / 180)   # right of image = negative in ENU/CCW
                cow_predicted_x = drone_x + math.cos(world_bearing) * full_distance
                cow_predicted_y = drone_y + math.sin(world_bearing) * full_distance  # + for ENU
                msg = Pose()
                # Set Position
                msg.position.x = cow_predicted_x
                msg.position.y = cow_predicted_y
                msg.position.z = num
                # Set Orientation (Quaternion: x, y, z, w)
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0
                msg.orientation.w = 1.0

                detections_pos.poses.append(msg)

        if len(detections_pos.poses) > 0:
            self.publisher.publish(detections_pos)

        # Show frame
        # cv2.imshow(name, annotated)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()