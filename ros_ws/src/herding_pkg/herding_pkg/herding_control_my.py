from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import cv2
import sys
import math

class HerdingControlMy(Node):
    def __init__(self):
        super().__init__('herding_control')

        if len(sys.argv) < 3:
            self.get_logger().error("Uso: ros2 run <pacote> <script> <namespace> <num_drones>")
            sys.exit(1)

        self.namespace = sys.argv[1]
        self.number_of_drones = int(sys.argv[2])
        number_of_drones = self.number_of_drones

        # --- Occupancy Grid ---
        self.grid_width = 100
        self.grid_height = 100
        self.grid_amplifier = 2
        self.grid = np.full((self.grid_height, self.grid_width), 0, dtype=np.uint8)

        # --- Goal position (world coordinates, meters) ---
        # This is where the cows should be herded TO.
        self.goal = np.array([0.0, 15.0])

        # --- APF tuning constants ---
        # Distance behind the cow (away from goal) where the drone should position itself
        self.herding_distance = 4.0  # meters

        # Cow repulsion radius: drones are strongly pushed away within this distance
        self.cow_repulsion_radius = 3.0  # meters

        # Drone-drone separation desired distance
        self.drone_separation_dist = 5.0  # meters

        # Force weights
        self.w_attractive = 1.5    # Weight for target-point attraction
        self.w_cow_repulsion = 2.0  # Weight for cow repulsion
        self.w_drone_repulsion = 1.0  # Weight for drone-drone repulsion

        # APF parameters (same style as reference calculate_formation_force: d, a, c)
        # For cow repulsion: strong push when inside cow_repulsion_radius
        self.cow_rep_a = 4.0
        self.cow_rep_c = 1.0

        # For drone-drone repulsion
        self.drone_rep_a = 4.0
        self.drone_rep_c = 2.0

        # --- ROS infrastructure ---
        self.drone_pos_dir = {}
        self.last_drone_pos_dir = {}
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
        self.move_timer = self.create_timer(0.5, self.move_drones)

        self.get_logger().info(f"HerdingControlMy: monitoring {number_of_drones} drones. Goal={self.goal}")

    # -------------------------------------------------------------------------
    # Core helpers
    # -------------------------------------------------------------------------

    def get_cows_pos(self):
        """Extract world-space cow positions from the occupancy grid (value > 180)."""
        cows_pos = []
        for index, value in np.ndenumerate(self.grid):
            if value > 180:
                py, px = index
                x = (px - (self.grid_width / 2)) / self.grid_amplifier
                y = (py - (self.grid_height / 2)) / self.grid_amplifier
                cows_pos.append(np.array([x, y]))
        return cows_pos

    def compute_drone_targets(self, cows_pos):
        """
        For each detected cow, compute the ideal drone position:
        a point `herding_distance` behind the cow (opposite side from the goal).

        Math:
            direction = normalize(cow - goal)   # vector pointing away from goal
            target    = cow + direction * herding_distance
        """
        targets = []
        for cow in cows_pos:
            diff = cow - self.goal
            dist = np.linalg.norm(diff)
            if dist < 2.0:
                # Cow is closer than 2m to the goal/target; consider it reached the goal
                continue
            direction = diff / dist
            target = cow + direction * self.herding_distance
            targets.append(target)
        return targets

    # -------------------------------------------------------------------------
    # APF force primitives
    # -------------------------------------------------------------------------

    def attractive_force(self, drone_pos, target):
        """
        Simple linear attraction toward a target point.
        F = (target - drone_pos)  (capped to unit vector, scaled by weight)
        """
        diff = target - drone_pos
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros(2)
        # Attract with magnitude proportional to distance (up to a cap)
        magnitude = min(dist, 2.0)
        return (diff / dist) * magnitude

    def repulsive_force(self, drone_pos, obstacle_pos, repulsion_radius, a, c):
        """
        APF repulsive force (same mathematical style as reference calculate_formation_force).
        Strong repulsion when dist < repulsion_radius, negligible when far.

            diff   = drone_pos - obstacle_pos   (points AWAY from obstacle)
            scalar = a * (1 - exp(-(dist - repulsion_radius) / c)) / (1 + dist)
            F      = scalar * direction

        When dist < repulsion_radius the exponent is positive → scalar is negative
        → force flips sign and pushes AWAY. When dist > repulsion_radius → near zero.
        We clamp to always be repulsive (non-negative scalar).
        """
        diff = drone_pos - obstacle_pos
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            # Exact overlap: push in random direction
            return np.array([a, 0.0])

        direction = diff / dist

        # Standard formation-force scalar (same as reference)
        numerator = a * (1.0 - np.exp(-(dist - repulsion_radius) / c))
        denominator = 1.0 + dist
        scalar = numerator / denominator

        # Negate so that when inside radius (scalar < 0) the force points away
        # and when outside radius (scalar > 0 approaching 0) we still repel slightly.
        # Using -scalar so that close distances yield a large positive repulsion.
        return -scalar * direction

    # -------------------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------------------

    def move_drones(self):
        if len(self.drone_pos_dir) < self.number_of_drones:
            return

        if not hasattr(self, '_scan_finished'):
            self._scan_finished = False

        if not self._scan_finished:
            if not hasattr(self, '_scan_initialized'):
                self._scan_initialized = True
                self._scan_progress = {}
                for name, pose in self.drone_pos_dir.items():
                    yaw = math.radians(self.get_yaw_degrees(pose.orientation))
                    self._scan_progress[name] = {
                        'start_yaw': yaw,
                        'current_yaw': yaw,
                        'initial_pos': np.array([pose.position.x, pose.position.y])
                    }
                self.get_logger().info("Starting initial 360-degree scan...")

            all_done = True
            for name, progress in self._scan_progress.items():
                pose = self.drone_pos_dir[name]
                rel_angle = progress['current_yaw'] - progress['start_yaw']
                if rel_angle < 2 * math.pi:
                    progress['current_yaw'] += 0.2
                    all_done = False
                
                # Keep drone in place by command to initial_pos
                target_pose = Pose()
                target_pose.position.x = float(progress['initial_pos'][0])
                target_pose.position.y = float(progress['initial_pos'][1])
                target_pose.position.z = pose.position.z
                self.drone_goto_pub_list[name].publish(target_pose)

                # Focus point rotates around initial_pos to complete 360 degrees
                focus_x = progress['initial_pos'][0] + 2.0 * math.cos(progress['current_yaw'])
                focus_y = progress['initial_pos'][1] + 2.0 * math.sin(progress['current_yaw'])
                focus_pose = Pose()
                focus_pose.position.x = float(focus_x)
                focus_pose.position.y = float(focus_y)
                focus_pose.position.z = pose.position.z
                self.drone_focusin_pub_list[name].publish(focus_pose)

            if all_done:
                self._scan_finished = True
                self.get_logger().info("360-degree scan complete. Starting herding algorithm.")
            return

        cows_pos = self.get_cows_pos()
        targets = self.compute_drone_targets(cows_pos)

        # Cache for visualization
        self._last_targets = targets

        for name, pose in self.drone_pos_dir.items():
            xi = np.array([pose.position.x, pose.position.y])
            total_force = np.zeros(2)

            # 1. Attractive force: every computed target point pulls every drone
            for target in targets:
                f = self.attractive_force(xi, target)
                total_force += self.w_attractive * f

            # 2. Repulsive force from cows: avoid entering cow_repulsion_radius
            # The closer the cow is to the final destination (goal), the more intense the repulsion force is.
            # When cows are at or near the final destination, we scale up the repulsion radius and weight
            # so that drones are pushed far away and do not nudge them beyond the goal.
            for cow in cows_pos:
                dist_to_goal = np.linalg.norm(cow - self.goal)
                factor = max(0.0, 1.0 - dist_to_goal / 10.0)
                r_rep = self.cow_repulsion_radius + 5.0 * factor
                w_rep = self.w_cow_repulsion * (1.0 + 4.0 * factor)

                f = self.repulsive_force(xi, cow, r_rep,
                                         self.cow_rep_a, self.cow_rep_c)
                total_force += w_rep * f

            # 3. Repulsive force from other drones (keep separation)
            for other_name, other_pose in self.drone_pos_dir.items():
                if other_name == name:
                    continue
                xj = np.array([other_pose.position.x, other_pose.position.y])
                f = self.repulsive_force(xi, xj, self.drone_separation_dist,
                                         self.drone_rep_a, self.drone_rep_c)
                total_force += self.w_drone_repulsion * f

            # Normalise to unit step
            force_mag = np.linalg.norm(total_force)
            if force_mag > 1e-6:
                unit_force = total_force / force_mag
            else:
                unit_force = np.zeros(2)

            # Move drone one step in the direction of the net force
            target_pose = Pose()
            target_pose.position.x = float(xi[0] + unit_force[0])
            target_pose.position.y = float(xi[1] + unit_force[1])
            self.drone_goto_pub_list[name].publish(target_pose)

            # Point camera in the direction of the nearest cow, or the net force vector if none detected
            focus_pose = Pose()
            if cows_pos:
                dists = [np.linalg.norm(cow - xi) for cow in cows_pos]
                nearest_cow = cows_pos[np.argmin(dists)]
                focus_pose.position.x = float(nearest_cow[0])
                focus_pose.position.y = float(nearest_cow[1])
            else:
                focus_pose.position.x = float(xi[0] + unit_force[0] * 10.0)
                focus_pose.position.y = float(xi[1] + unit_force[1] * 10.0)
            focus_pose.position.z = pose.position.z  # keep same altitude focus
            self.drone_focusin_pub_list[name].publish(focus_pose)

    # -------------------------------------------------------------------------
    # Visualization
    # -------------------------------------------------------------------------

    def world_to_grid(self, x, y):
        """Convert world coordinates (meters) to grid pixel indices."""
        px = int(x * self.grid_amplifier + self.grid_width / 2)
        py = int(y * self.grid_amplifier + self.grid_height / 2)
        return px, py

    def create_map(self):
        display_img = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB)

        # Draw goal: green filled circle
        gx, gy = self.world_to_grid(self.goal[0], self.goal[1])
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            cv2.circle(display_img, (gx, gy), 1, (0, 255, 0), -1)  # green

        # Draw computed drone target positions: yellow dots
        if hasattr(self, '_last_targets'):
            for target in self._last_targets:
                tx, ty = self.world_to_grid(target[0], target[1])
                if 0 <= tx < self.grid_width and 0 <= ty < self.grid_height:
                    cv2.circle(display_img, (tx, ty), 1, (0, 255, 255), -1)  # yellow

        # Draw drones: blue dots (existing function)
        self.draw_drones_pos(self.drone_pos_dir, display_img)

        display_img = cv2.flip(display_img, 0)
        resized_img = cv2.resize(display_img, (500, 500), interpolation=cv2.INTER_NEAREST)

        cv2.imshow("Herding Control (APF) - Occupancy Grid", resized_img)
        cv2.waitKey(1)

    # -------------------------------------------------------------------------
    # Unchanged helpers from reference
    # -------------------------------------------------------------------------

    def get_yaw_degrees(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw_radians = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_radians)

    def draw_drones_pos(self, drone_pos_dir, image):
        radius = 1
        color = (255, 0, 0)   # blue in BGR

        for name, pose in drone_pos_dir.items():
            x_idx = int((pose.position.x * self.grid_amplifier) + self.grid_width / 2)
            y_idx = int((pose.position.y * self.grid_amplifier) + self.grid_height / 2)
            cv2.circle(image, (x_idx, y_idx), radius, color, -1)

    def cow_callback(self, msg):
        drone_pos = self.drone_pos_dir[self.namespace + str(int(msg.poses[0].position.z))]
        drone_x_idx = int((drone_pos.position.x * self.grid_amplifier) + self.grid_width / 2)
        drone_y_idx = int((drone_pos.position.y * self.grid_amplifier) + self.grid_height / 2)
        drone_qz = drone_pos.orientation.z
        drone_qw = drone_pos.orientation.w
        drone_yaw = 2 * math.atan2(drone_qz, drone_qw)

        poses = []
        for obj in msg.poses:
            x_idx = int((obj.position.x * self.grid_amplifier) + self.grid_width / 2)
            y_idx = int((obj.position.y * self.grid_amplifier) + self.grid_height / 2)
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
        for px, py in object_set:
            grid[py][px] = min(grid[py][px] + 150, 255)

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
    node = HerdingControlMy()

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