import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import yaml
import math
import os
import random

# Scale factor: pixels to meters (must match nav_graph conversion)
PIXEL_TO_METER = 1.0 / 30.0  # 0.033 meters per pixel

class Robot:
    def __init__(self, id, x, y, theta):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta # radians, 0 is right, pi/2 is up? user said "start by pointing up"
        # usually 0 is East (Right), pi/2 is North (Up). So initial theta should be -pi/2 or pi/2 depending on coord system.
        # In image coords: X right, Y down. So "Up" is -Y.
        # 0 is (1,0) [Right]. pi/2 is (0,1) [Down]. -pi/2 is (0,-1) [Up].
        # So "pointing up" means theta = -pi/2.
        
        self.v = 0.0 # linear speed
        self.w = 0.0 # angular speed
        self.radius = 10 # pixels, ~0.17m at 30px/m for tighter collision
        self.in_collision = False
        
        # LIDAR parameters
        self.num_rays = 12  # 360 / 30 = 12 rays
        self.lidar_max_range = 500.0  # pixels (~16.7m at 30px/m)

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('start_config', '')
        self.declare_parameter('num_robots_override', 0) # 0 means use config
        self.declare_parameter('map_img_path', '')

        self.map_yaml_path = self.get_parameter('map_yaml').value
        self.start_config_path = self.get_parameter('start_config').value
        self.map_img_path = self.get_parameter('map_img_path').value
        
        self.get_logger().info(f'Loading config from: {self.start_config_path}')

        self.load_map()
        self.load_robots()
        
        self.dt = 0.1
        self.render_counter = 0
        self.render_every = 1  # Render every Nth tick to keep loop fast
        self.create_timer(self.dt, self.loop)
        
        # Subscribers and Publishers
        self.subs = []
        self.pubs = []
        for r in self.robots:
            sub = self.create_subscription(
                Twist,
                f'/robot_{r.id}/cmd_vel',
                lambda msg, robot=r: self.cmd_vel_callback(msg, robot),
                10
            )
            self.subs.append(sub)
            
            pub = self.create_publisher(PoseStamped, f'/robot_{r.id}/pose', 10)
            self.pubs.append(pub)
            
            # LIDAR publisher
            if not hasattr(self, 'lidar_pubs'):
                self.lidar_pubs = []
            
            lpub = self.create_publisher(LaserScan, f'/robot_{r.id}/scan', 10)
            self.lidar_pubs.append(lpub)
        
        self.get_logger().info(f'Simulator started with {len(self.robots)} robots')

        # GUI Setup
        try:
            cv2.namedWindow("Robot Simulator")
            cv2.setMouseCallback("Robot Simulator", self.mouse_callback)
            self.gui_available = True
            self.show_lidar = True

            # Toolbar button definitions
            # Each button: (label, color_on, color_off_or_None, callback)
            # color_off=None means non-toggle (always color_on)
            self.btn_margin = 6          # margin around the toolbar and between buttons
            self.btn_height = 32         # button height
            self.btn_font = cv2.FONT_HERSHEY_SIMPLEX
            self.btn_font_scale = 0.55
            self.btn_font_thickness = 2
            self.buttons = []            # populated by _build_toolbar
            self._build_toolbar()
        except Exception as e:
            self.get_logger().warn(f"GUI initialization failed: {e}. Running in headless mode.")
            self.gui_available = False

    def _build_toolbar(self):
        """Compute button rectangles with uniform margins."""
        m = self.btn_margin
        h = self.btn_height
        pad_x = 12  # horizontal text padding inside button

        btn_defs = [
            ('EXIT',  (0, 0, 200), None,            self._on_exit),
            ('LIDAR', (0, 160, 0), (100, 100, 100), self._on_toggle_lidar),
        ]

        self.buttons = []
        x_cursor = m  # start after left margin
        y_top = m
        y_bot = m + h

        for label, color_on, color_off, cb in btn_defs:
            (tw, th), _ = cv2.getTextSize(label, self.btn_font,
                                          self.btn_font_scale,
                                          self.btn_font_thickness)
            btn_w = tw + pad_x * 2
            self.buttons.append({
                'label': label,
                'x1': x_cursor, 'y1': y_top,
                'x2': x_cursor + btn_w, 'y2': y_bot,
                'color_on': color_on,
                'color_off': color_off,
                'callback': cb,
                'text_x': x_cursor + pad_x,
                'text_y': y_bot - (h - th) // 2,
            })
            x_cursor += btn_w + m  # advance by button width + gap

        self.toolbar_height = y_bot + m  # total toolbar height including bottom margin

    def _on_exit(self):
        self.get_logger().info('Exit button clicked')
        rclpy.shutdown()

    def _on_toggle_lidar(self):
        self.show_lidar = not self.show_lidar
        self.get_logger().info(f"LIDAR display {'ON' if self.show_lidar else 'OFF'}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for btn in self.buttons:
                if btn['x1'] <= x <= btn['x2'] and btn['y1'] <= y <= btn['y2']:
                    btn['callback']()
                    break

    def draw_toolbar(self, img):
        """Draw the toolbar background and all buttons with uniform spacing."""
        # Toolbar background
        cv2.rectangle(img, (0, 0),
                      (img.shape[1], self.toolbar_height),
                      (50, 50, 50), -1)

        for btn in self.buttons:
            # Pick colour based on toggle state
            if btn['color_off'] is not None:
                # Toggle button – check corresponding state
                if btn['label'] == 'LIDAR':
                    color = btn['color_on'] if self.show_lidar else btn['color_off']
                else:
                    color = btn['color_on']
            else:
                color = btn['color_on']

            cv2.rectangle(img,
                          (btn['x1'], btn['y1']),
                          (btn['x2'], btn['y2']),
                          color, -1)
            cv2.putText(img, btn['label'],
                        (btn['text_x'], btn['text_y']),
                        self.btn_font, self.btn_font_scale,
                        (255, 255, 255), self.btn_font_thickness)

    def load_map(self):
        with open(self.map_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            
        self.vertices = data['levels']['L1']['vertices'] 
        # vertices: [x, y, z, name]
        
        walls_data = data['levels']['L1']['walls']
        # walls: [start_idx, end_idx, params]
        
        self.walls = []
        for w in walls_data:
            idx1 = w[0]
            idx2 = w[1]
            p1 = self.vertices[idx1] # [x, y, ...]
            p2 = self.vertices[idx2]
            self.walls.append(((p1[0], p1[1]), (p2[0], p2[1])))

        # Load background image
        if os.path.exists(self.map_img_path):
             self.bg_img = cv2.imread(self.map_img_path)
        else:
            # Create a white canvas based on vertices bounds
            xs = [v[0] for v in self.vertices]
            ys = [v[1] for v in self.vertices]
            w = int(max(xs) + 100)
            h = int(max(ys) + 100)
            self.bg_img = np.ones((h, w, 3), dtype=np.uint8) * 255
            # Draw walls on it since we don't have the fancy rendering
            for p1, p2 in self.walls:
                pt1 = (int(p1[0]), int(p1[1]))
                pt2 = (int(p2[0]), int(p2[1]))
                cv2.line(self.bg_img, pt1, pt2, (0, 0, 0), 2)
        
        # Load nav_graph lanes for visualization from the same building file
        self.nav_lanes = []
        self.nav_vertices = []
        self.load_nav_graph_from_building(data)
    
    def load_nav_graph_from_building(self, data):
        """Load nav_graph lanes directly from the building YAML data."""
        try:
            level = data['levels']['L1']
            lanes = level.get('lanes', [])
            vertices = level['vertices']
            
            # Get all vertex indices used in lanes
            used_indices = set()
            for lane in lanes:
                used_indices.add(lane[0])
                used_indices.add(lane[1])
            
            # Build vertex lookup (only for used vertices)
            vertex_positions = {}
            for idx in used_indices:
                if idx < len(vertices):
                    v = vertices[idx]
                    vertex_positions[idx] = (v[0], v[1])
            
            # Store lanes as pairs of pixel coordinates
            for lane in lanes:
                v1_idx, v2_idx = lane[0], lane[1]
                if v1_idx in vertex_positions and v2_idx in vertex_positions:
                    p1 = vertex_positions[v1_idx]
                    p2 = vertex_positions[v2_idx]
                    self.nav_lanes.append((p1, p2))
            
            self.get_logger().info(f'Loaded {len(self.nav_lanes)} nav lanes from building file')
        except Exception as e:
            self.get_logger().warn(f'Could not load nav_graph from building: {e}')
                
    def load_robots(self):
        self.robots = []
        
        # Load from config file if present
        start_poses = []
        num_robots = 100 # Default requirement
        
        if self.start_config_path and os.path.exists(self.start_config_path):
            self.get_logger().info(f'Config file exists: {self.start_config_path}')
            with open(self.start_config_path, 'r') as f:
                conf = yaml.safe_load(f)
                if 'robots' in conf:
                   num_robots = len(conf['robots'])
                   for r_conf in conf['robots']:
                       start_poses.append((r_conf['x'], r_conf['y'], r_conf['theta']))
                   self.get_logger().info(f'Loaded {len(start_poses)} spawn positions from config: {start_poses[:3]}...')
        else:
            self.get_logger().warn(f'Config file not found: {self.start_config_path}')
        
        override = self.get_parameter('num_robots_override').value
        if override > 0:
            num_robots = override
            
        # Spawn logic
        for i in range(num_robots):
            if i < len(start_poses):
                x, y, th = start_poses[i]
                # If loading from config, use index + 1 if IDs are implicit, 
                # but we are just taking positions. 
                # We will assign ID = i + 1.
            else:
                # Random spawn if not specified (avoid walls ideally, but simplest first)
                # We need a spawn function that checks for collision
                x, y = self.find_safe_spawn()
                th = -math.pi / 2 # Pointing UP (Negative Y)
            
            # Use 1-based indexing for ID
            self.robots.append(Robot(i + 1, x, y, th))
            
    def find_safe_spawn(self):
        # Determine bounds
        h, w, _ = self.bg_img.shape
        # Try random positions
        for _ in range(1000):
            rx = random.uniform(0, w)
            ry = random.uniform(0, h)
            # Check collision with walls and other robots
            if not self.check_collision_point(rx, ry, radius=15, exclude_robot_id=None):
                return rx, ry
        return 0, 0 # Fallback

    def cmd_vel_callback(self, msg, robot):
        robot.v = msg.linear.x
        robot.w = msg.angular.z

    def check_collision_point(self, x, y, radius, exclude_robot_id=None):
        # Check against all walls
        for p1, p2 in self.walls:
            # Segment point distance
            dist = self.point_segment_distance(x, y, p1[0], p1[1], p2[0], p2[1])
            if dist < radius:
                return True
        
        # Check against other robots
        for other in self.robots:
            if exclude_robot_id is not None and other.id == exclude_robot_id:
                continue
            # Distance between centers
            dist = math.hypot(x - other.x, y - other.y)
            # Collision if distance < sum of radii
            if dist < (radius + other.radius):
                return True
        return False

    def point_segment_distance(self, px, py, x1, y1, x2, y2):
        # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        l2 = (x1 - x2)**2 + (y1 - y2)**2
        if l2 == 0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2
        t = max(0, min(1, t))
        
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        return math.hypot(px - proj_x, py - proj_y)

    def ray_segment_intersection(self, ray_origin, ray_dir, seg_start, seg_end):
        """Find intersection distance between ray and line segment.
        Returns distance to intersection or None if no intersection."""
        ox, oy = ray_origin
        dx, dy = ray_dir
        x1, y1 = seg_start
        x2, y2 = seg_end
        
        # Ray: P = O + t * D (t >= 0)
        # Segment: Q = S1 + u * (S2 - S1) (0 <= u <= 1)
        
        denom = dx * (y2 - y1) - dy * (x2 - x1)
        if abs(denom) < 1e-10:
            return None  # Parallel
        
        t = ((x1 - ox) * (y2 - y1) - (y1 - oy) * (x2 - x1)) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
        
        if t >= 0 and 0 <= u <= 1:
            return t  # Distance along ray
        return None

    def ray_circle_intersection(self, ray_origin, ray_dir, circle_center, circle_radius):
        """Find intersection distance between ray and circle.
        Returns distance to nearest intersection or None if no intersection."""
        ox, oy = ray_origin
        dx, dy = ray_dir
        cx, cy = circle_center
        
        # Vector from ray origin to circle center
        fx = ox - cx
        fy = oy - cy
        
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - circle_radius * circle_radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return None  # No intersection
        
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        # Return nearest positive intersection
        if t1 >= 0:
            return t1
        elif t2 >= 0:
            return t2
        return None

    def cast_ray(self, robot, angle):
        """Cast a ray from robot at given angle (world frame) and return distance to nearest obstacle."""
        ray_origin = (robot.x, robot.y)
        ray_dir = (math.cos(angle), math.sin(angle))
        
        min_dist = robot.lidar_max_range
        
        # Check against all walls
        for p1, p2 in self.walls:
            dist = self.ray_segment_intersection(ray_origin, ray_dir, p1, p2)
            if dist is not None and dist < min_dist:
                min_dist = dist
        
        # Check against other robots (treat as circles)
        for other in self.robots:
            if other.id == robot.id:
                continue
            dist = self.ray_circle_intersection(ray_origin, ray_dir, 
                                                 (other.x, other.y), other.radius)
            if dist is not None and dist < min_dist:
                min_dist = dist
        
        return min_dist

    def compute_lidar_scan(self, robot):
        """Compute LIDAR scan for a robot with 12 rays at 30 degree increments."""
        ranges = []
        angle_increment = math.radians(30)  # 30 degrees between rays
        
        for i in range(robot.num_rays):
            # Ray angle in world frame (robot theta + ray offset)
            ray_angle = robot.theta + i * angle_increment
            distance = self.cast_ray(robot, ray_angle)
            ranges.append(distance)
        
        return ranges

    def publish_lidar(self, robot, ranges):
        """Publish LaserScan message for a robot."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f'robot_{robot.id}/base_link'
        
        # LIDAR parameters
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi - math.radians(30)  # Full 360 minus one increment
        scan.angle_increment = math.radians(30)  # 30 degrees
        scan.time_increment = 0.0
        scan.scan_time = self.dt
        scan.range_min = 0.0
        scan.range_max = robot.lidar_max_range * PIXEL_TO_METER  # Convert to meters
        
        # Convert ranges from pixels to meters
        scan.ranges = [r * PIXEL_TO_METER for r in ranges]
        scan.intensities = []  # Not used
        
        idx = robot.id - 1
        if 0 <= idx < len(self.lidar_pubs):
            self.lidar_pubs[idx].publish(scan)

    def loop(self):
        # 1. Update Physics
        for r in self.robots:
            # Simple kinematics
            # x' = x + v * cos(theta) * dt
            # y' = y + v * sin(theta) * dt
            # th' = th + w * dt
            
            # Map coords: Y is likely down, so "Up" is -Y.
            # If standard math: Up is +Y.
            # Let's assume standard angles 0=Right, PI/2=Down (screen coords).
            # So v * cos(theta) moves in X
            # v * sin(theta) moves in Y
            
            # Note: User provided vertices which look like pixel coords approx.
            # 1 meter might be ~20 pixels? User didn't specify scale.
            # I will apply v directly (1 unit/sec). User can scale cmd_vel.
            
            new_x = r.x + r.v * math.cos(r.theta) * self.dt
            new_y = r.y + r.v * math.sin(r.theta) * self.dt
            new_th = r.theta + r.w * self.dt
            
            # Update position and orientation
            r.x = new_x
            r.y = new_y
            r.theta = new_th
            
            # Compute LIDAR scan
            lidar_ranges = self.compute_lidar_scan(r)
            r.lidar_ranges = lidar_ranges  # Store for visualization
            
            # Collision detection based on LIDAR: if any ray length < 1 pixel
            r.in_collision = any(ray_length < 1.0 for ray_length in lidar_ranges)
            
            if r.in_collision:
                self.get_logger().warn(f'Robot {r.id} collision detected (LIDAR ray < 1 pixel)')
            
            # Publish Pose (in METERS, not pixels)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = r.x * PIXEL_TO_METER
            pose_msg.pose.position.y = r.y * PIXEL_TO_METER
            pose_msg.pose.position.z = 0.0
            # Quaternion from theta (Yaw)
            # qx=0, qy=0, qz=sin(th/2), qw=cos(th/2)
            pose_msg.pose.orientation.z = math.sin(r.theta / 2.0)
            pose_msg.pose.orientation.w = math.cos(r.theta / 2.0)
            
            # Access publisher by index? `r.id` starts at 1. `self.pubs` is 0-indexed.
            # We must map r.id to index. Valid IDs: 1..N. Index: 0..N-1.
            idx = r.id - 1
            if 0 <= idx < len(self.pubs):
                self.pubs[idx].publish(pose_msg)
            
            # Publish LIDAR scan
            self.publish_lidar(r, lidar_ranges)

        # 2. Render (only every Nth tick to keep loop fast for cmd_vel processing)
        self.render_counter += 1
        if self.gui_available and (self.render_counter % self.render_every == 0):
            try:
                img = self.bg_img.copy()
                
                # Draw nav_graph lanes (expected robot paths) in green
                for p1, p2 in self.nav_lanes:
                    pt1 = (int(p1[0]), int(p1[1]))
                    pt2 = (int(p2[0]), int(p2[1]))
                    cv2.line(img, pt1, pt2, (0, 200, 0), 2)  # Green lanes
                
                # Draw nav_graph vertices as small circles
                for x, y, name in self.nav_vertices:
                    pt = (int(x), int(y))
                    cv2.circle(img, pt, 5, (0, 150, 0), -1)  # Dark green dots
                    if name:  # Draw vertex name
                        cv2.putText(img, name, (int(x)+7, int(y)-5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 100, 0), 1)
                
                # Draw toolbar buttons
                self.draw_toolbar(img)
                
                for r in self.robots:
                    self.draw_robot(img, r)
                    
                cv2.imshow("Robot Simulator", img)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"GUI render failed: {e}")
                self.gui_available = False

    def draw_robot(self, img, robot):
        # Squircle shape: Tombstone
        # Center: r.x, r.y
        # Orientation: r.theta
        
        color = (255, 255, 255) # White
        border_color = (0, 0, 0) # Black
        if robot.in_collision:
            border_color = (0, 0, 255) # Red (BGR)
            
        # Create a local canvas for the robot, rotate it, and paste?
        # Or draw primitives.
        
        # A tombstone is a rectangle with a circle on top.
        # "Front" is circular.
        # Robot is pointing at r.theta.
        
        # Size
        w = robot.radius * 2
        h = robot.radius * 2
        
        # We need to draw a rotated shape.
        # Best way in OpenCV: define points of the polygon/shape around (0,0), rotate, translate, fill.
        
        # Shape definition (facing right [0 radians]):
        # "One side is circular... front side".
        # Let's say front is +X.
        # So right side is semi-circle. Left side is flat.
        
        # Rectangle part: (-r, -r) to (0, r)
        # Semicircle center (0, 0), radius r, from -90 to 90 deg.
        # Combined:
        # P1: (-r, -r)
        # P2: (0, -r)
        # Arc from P2 to P3(0, r)
        # P3: (0, r)
        # P4: (-r, r)
        # Close to P1.
        
        # Vertices for polygon (approximating arc)
        pts = []
        pts.append((-robot.radius, -robot.radius)) # Top-Left (local)
        
        # Arc points
        # 0 is center x.
        for angle in range(-90, 91, 15): # Deg
            rad = math.radians(angle)
            px = robot.radius * math.cos(rad) * 0.5 # Flatten the circle slightly? No, full circle.
            # Actually center of circle part is at 0,0?
            # If the shape is Squircle, let's keep it simple.
            # Front (X+) is curvy.
            px = 0 + robot.radius * math.cos(rad)
            py = 0 + robot.radius * math.sin(rad)
            pts.append((px, py))
            
        pts.append((-robot.radius, robot.radius)) # Bottom-Left (local)
        
        # Now rotate and translate
        transformed_pts = []
        cos_th = math.cos(robot.theta)
        sin_th = math.sin(robot.theta)
        
        for px, py in pts:
            # Rotate
            rx = px * cos_th - py * sin_th
            ry = px * sin_th + py * cos_th
            # Translate
            tx = rx + robot.x
            ty = ry + robot.y
            transformed_pts.append([int(tx), int(ty)])
            
        pts_np = np.array(transformed_pts, np.int32)
        pts_np = pts_np.reshape((-1, 1, 2))
        
        # Draw filled
        cv2.fillPoly(img, [pts_np], color)
        # Draw border
        cv2.polylines(img, [pts_np], True, border_color, 2)
        
        # Text
        # "number always facing right side up"
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = str(robot.id)
        text_size = cv2.getTextSize(text, font, 0.4, 1)[0]
        text_x = int(robot.x - text_size[0] / 2)
        text_y = int(robot.y + text_size[1] / 2)
        
        cv2.putText(img, text, (text_x, text_y), font, 0.4, (0,0,0), 1)

        # Draw LIDAR rays
        if hasattr(self, 'show_lidar') and self.show_lidar and hasattr(robot, 'lidar_ranges'):
            angle_increment = math.radians(30)
            for i, ray_range in enumerate(robot.lidar_ranges):
                ray_angle = robot.theta + i * angle_increment
                end_x = robot.x + ray_range * math.cos(ray_angle)
                end_y = robot.y + ray_range * math.sin(ray_angle)
                pt_start = (int(robot.x), int(robot.y))
                pt_end = (int(end_x), int(end_y))
                # Color: green for long range, red for close hits
                ratio = ray_range / robot.lidar_max_range
                r_color = int(255 * (1.0 - ratio))
                g_color = int(255 * ratio)
                cv2.line(img, pt_start, pt_end, (0, g_color, r_color), 1)
                # Small dot at hit point
                cv2.circle(img, pt_end, 2, (0, g_color, r_color), -1)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
