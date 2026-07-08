from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import cv2
import sys
import math

class HerdingControl(Node):
    def __init__(self):
        super().__init__('herding_control')

        if len(sys.argv) < 3:
            self.get_logger().error("Uso: ros2 run <pacote> <script> <namespace> <num_drones>")
            sys.exit(1)

        self.namespace = sys.argv[1]
        number_of_drones = int(sys.argv[2])

        self.grid_width = 100
        self.grid_height = 100
        self.grid_amplifier = 2
        self.grid = np.full((self.grid_height, self.grid_width), 0, dtype=np.uint8)

        self.drone_pos_dir = {}
        self.last_drone_pos_dir = {}
        self.last_cow_group_pos = None
        self.drone_pos_sub_list = {}
        self.drone_goto_pub_list = {}
        self.drone_focusin_pub_list = {}

        for i in range(number_of_drones):
            name = self.namespace + str(i)
            self.drone_pos_sub_list[name] = self.create_subscription(
                Pose, 
                f'{name}/gt_pose', 
                lambda msg, n=name: self.drone_callback(msg, n), 
                10)
            self.drone_goto_pub_list[name] = self.create_publisher(
                Pose,
                f'{name}/goto',
                10)
            self.drone_focusin_pub_list[name] = self.create_publisher(
                Pose,
                f'{name}/focusin',
                10)

        self.cow_position_sub = self.create_subscription(
            PoseArray, '/cows_pos', self.cow_callback, 10)

        self.timer = self.create_timer(0.5, self.create_map)

        #self.apf_timer = self.create_timer(10, self.create_apf_map)

        self.move_timer = self.create_timer(0.5, self.move_drones)

        self.get_logger().info(f"Monitoramento de {number_of_drones} drones iniciado.")

    
    def get_cows_pos(self):
        cows_pos = []
        for index, value in np.ndenumerate(self.grid):
            if value > 180:
                py, px = index
                x = (px - (self.grid_width / 2)) / self.grid_amplifier
                y = (py - (self.grid_height / 2)) / self.grid_amplifier
                cows_pos.append(np.array([x, y]))
        return cows_pos


    def move_drones(self):
        cows_pos = self.get_cows_pos()
        #print(cows_pos)

        if len(cows_pos) > 0:
            center_of_cows = np.mean(cows_pos, axis=0)
        else:
            center_of_cows = np.array([0.0, 0.0])

        center_of_cows_pose = Pose()
        center_of_cows_pose.position.x = float(center_of_cows[0])
        center_of_cows_pose.position.y = float(center_of_cows[1])

        # Calculate cows movement direction (velocity vector)
        prev_center = None
        if self.last_cow_group_pos is not None:
            prev_center = np.array([
                self.last_cow_group_pos.position.x,
                self.last_cow_group_pos.position.y
            ])
            cows_velocity = center_of_cows - prev_center
        else:
            cows_velocity = np.array([0.0, 0.0])

        self.last_cow_group_pos = center_of_cows_pose
        #print(center_of_cows_pose)

        for name, pose in self.drone_pos_dir.items():
            xi = np.array([pose.position.x, pose.position.y])
            total_force = np.array([0.0, 0.0])

            # Animal-drone interaction
            for xj in cows_pos:
                f = self.calculate_formation_force(xj, xi, d=4.0, a=4.0, c=2.0)
                total_force += f

            # Drone-drone interaction
            for other_name, other_pose in self.drone_pos_dir.items():
                if name != other_name:
                    xj = np.array([other_pose.position.x, other_pose.position.y])
                    if np.linalg.norm(xi - xj) <= 500.0:
                        f = self.calculate_formation_force(xj, xi, d=5.0, a=4.0, c=2.0)
                        total_force += f

            # Add force based on cows' movement direction
            # This force will be proportional to the cows_velocity vector
            # You can tune the scale as needed (e.g., 0.5)
            if np.linalg.norm(cows_velocity) > 0:
                cows_velocity_force = 0.5 * cows_velocity / (np.linalg.norm(cows_velocity) + 1e-6)
                total_force += cows_velocity_force

            #Sum all forces and make a resulting vector with size 1
            force_mag = np.linalg.norm(total_force)
            if force_mag > 0:
                total_force = total_force / force_mag

            target_pose = Pose()
            target_pose.position.x = float(xi[0] + total_force[0])
            target_pose.position.y = float(xi[1] + total_force[1])

            #print(name)
            #print(pose.position)
            #print(target_pose)
            #print(total_force)
            #print(force_mag)
            self.drone_goto_pub_list[name].publish(target_pose)
            self.drone_focusin_pub_list[name].publish(center_of_cows_pose)


    def calculate_formation_force(self, xi, xj, d, a, c):
        xi = np.asarray(xi)
        xj = np.asarray(xj)
        
        # Calculate the vector difference (xi - xj)
        diff = xi - xj
        
        # Calculate the Euclidean distance ||xi - xj||
        dist = np.linalg.norm(diff)
        
        # Handle the case where xi and xj are at the exact same location
        # to avoid potential issues, though the denominator (1 + dist) prevents division by zero.
        if dist == 0:
            return np.zeros_like(xi)
        
        # Scalar term of Equation (3):
        # [a * (1 - exp(-(dist - d) / c))] / (1 + dist)
        numerator = a * (1 - np.exp(-(dist - d) / c))
        denominator = 1 + dist
        
        scalar_factor = numerator / denominator
        
        # Final force vector
        return scalar_factor * diff

    def create_apf_map(self):
        grid = np.full((30 -1, 30 -1), 0, dtype=np.int8)
        grid = self.grid[30:60][30:60]
        grid = np.pad(grid, pad_width=1, mode='constant', constant_values=255)

        old_grid = grid

        for iterate in range(100):
            for index, value in np.ndenumerate(self.grid):
                x, y = index
                if grid[x][y] < 20 and x > 0 and x < 30 - 1 and y > 0 and y< 30-1:
                    grid[x][y] = (old_grid[x +1][y] + old_grid[x -1][y] + old_grid[x][y +1] + old_grid[x][y -1]) / 4
            old_grid = grid

        display_img = cv2.flip(~grid, 0)
        resized_img = cv2.resize(display_img, (500, 500), interpolation=cv2.INTER_NEAREST)

        cv2.imshow("Herding Control - APF Grid", resized_img)
        cv2.waitKey(1)
        

    def create_map(self):
        display_img = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB)
        self.draw_drones_pos(self.drone_pos_dir, display_img)
        display_img = cv2.flip(display_img, 0)

        resized_img = cv2.resize(display_img, (500, 500), interpolation=cv2.INTER_NEAREST)

        cv2.imshow("Herding Control - Occupancy Grid", resized_img)
        cv2.waitKey(1)

    def get_yaw_degrees(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Calculate Yaw (heading) using the standard formula
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        
        yaw_radians = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to Degrees
        yaw_degrees = math.degrees(yaw_radians)
        
        return yaw_degrees

    def draw_drones_pos(self, drone_pos_dir, image):
        radius = 1             # Radius in pixels
        color = (255, 0, 0)      # BGR color: (Blue, Green, Red) -> Green
        thickness = -1          # Border thickness (use -1 to fill the circle)
        startAngle = -55         # Starting angle of the slice (in degrees)
        endAngle = 55          # Ending angle of the slice (in degrees)
        color_rad = (0, 0, 40)    # Blue in BGR
        thickness_rad = 1       # -1 fills the slice; positive values draw an arc
        axes = (12, 12)      # Radius in both directions (makes it a circle)

        for name, pose in drone_pos_dir.items():
            x_idx = int((pose.position.x *self.grid_amplifier) + self.grid_width / 2)
            y_idx = int((pose.position.y *self.grid_amplifier) + self.grid_height / 2)

            center = (x_idx, y_idx)
            angle = self.get_yaw_degrees(pose.orientation)
            radius_val = axes[0]

            start_rad = math.radians(angle + startAngle)
            end_rad = math.radians(angle + endAngle)

            start_point = (
                int(center[0] + radius_val * math.cos(start_rad)),
                int(center[1] + radius_val * math.sin(start_rad))
            )
            end_point = (
                int(center[0] + radius_val * math.cos(end_rad)),
                int(center[1] + radius_val * math.sin(end_rad))
            )

            cv2.circle(image, center, radius, color, thickness)
            #cv2.ellipse(image, center, axes, angle, startAngle, endAngle, color_rad, thickness_rad)
            
            #cv2.line(image, center, start_point, color_rad, thickness_rad)
            #cv2.line(image, center, end_point, color_rad, thickness_rad)

    def cow_callback(self, msg):
        drone_pos = self.drone_pos_dir[self.namespace + str(int(msg.poses[0].position.z))]
        drone_x_idx = int((drone_pos.position.x *self.grid_amplifier) + self.grid_width / 2)
        drone_y_idx = int((drone_pos.position.y *self.grid_amplifier) + self.grid_height / 2)
        drone_qz = drone_pos.orientation.z
        drone_qw = drone_pos.orientation.w
        drone_yaw = 2 * math.atan2(drone_qz, drone_qw)  # radians

        poses = []
        for obj in msg.poses:
            x_idx = int((obj.position.x *self.grid_amplifier)  + self.grid_width / 2)
            y_idx = int((obj.position.y *self.grid_amplifier)  + self.grid_height / 2)
            poses.append((x_idx, y_idx))

        self.update_occupancy_grid(
            self.grid,
            (drone_x_idx, drone_y_idx),
            drone_yaw,
            2,
            20 * self.grid_amplifier,
            poses
        )
        
    def drone_callback(self, msg, namespace):
        self.drone_pos_dir[namespace] = msg

    def update_occupancy_grid(self, grid, cam_pos, cam_angle, fov, max_dist, detections):
        grid_size = len(grid)
        cx, cy = cam_pos
        num_steps = 180
        angle_step = fov / num_steps
        depth_buffer = {i: (float(max_dist), None) for i in range(num_steps)}

        for obj_x, obj_y in detections:
            dx, dy = obj_x - cx, obj_y - cy
            dist = math.sqrt(dx**2 + dy**2)
            angle_to_obj = math.atan2(dy, dx)
            rel_angle = (angle_to_obj - cam_angle + math.pi) % (2 * math.pi) - math.pi
            
            if abs(rel_angle) <= fov / 2 and dist <= max_dist:
                bin_idx = int((rel_angle + fov / 2) / angle_step)
                bin_idx = max(0, min(num_steps - 1, bin_idx))
                if dist < depth_buffer[bin_idx][0]:
                    depth_buffer[bin_idx] = (dist, (obj_x, obj_y))

        clear_set = set()
        object_set = set()

        for i in range(num_steps):
            target_dist, obj_coords = depth_buffer[i]
            curr_rel_angle = (-fov / 2) + (i * angle_step)
            abs_angle = cam_angle + curr_rel_angle
            
            if obj_coords:
                x1, y1 = int(obj_coords[0]), int(obj_coords[1])
                is_hit = True
            else:
                x1 = int(cx + target_dist * math.cos(abs_angle))
                y1 = int(cy + target_dist * math.sin(abs_angle))
                is_hit = False

            line_cells = self.bresenham_line(int(cx), int(cy), x1, y1)
            for idx, (px, py) in enumerate(line_cells):
                if 0 <= px < grid_size and 0 <= py < grid_size:
                    if idx < len(line_cells) - 1:
                        clear_set.add((px, py))
                    elif is_hit:
                        object_set.add((px, py))
                    else:
                        clear_set.add((px, py))

        for px, py in clear_set:
            grid[py][px] = max(grid[py][px] - 30, 0)
            #grid[py][px] = 0
        for px, py in object_set:
            grid[py][px] = min(grid[py][px] + 150, 255)
            #grid[py][px] = 0

    def bresenham_line(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

def main(args=None):
    rclpy.init(args=args)
    node = HerdingControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()