import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import random
import math
import yaml
import sys
import time

class SequentialNav(Node):
    def __init__(self):
        super().__init__('sequential_nav')
        
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # Load map to get vertices for goals
        self.load_vertices('/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.building.yaml')
        
        self.pubs = {}
        self.subs = {}
        self.collision_subs = {}
        self.poses = {}
        
        for i in range(1, self.num_robots + 1):
            # Publisher
            pub = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10)
            self.pubs[i] = pub
            
            # Subscriber Pose
            sub = self.create_subscription(
                PoseStamped, 
                f'/robot_{i}/pose',
                lambda msg, robot_id=i: self.pose_callback(msg, robot_id),
                10
            )
            self.subs[i] = sub
            
            # Subscriber Collision
            csub = self.create_subscription(
                Bool,
                f'/robot_{i}/collision',
                lambda msg, robot_id=i: self.collision_callback(msg, robot_id),
                10
            )
            self.collision_subs[i] = csub
            
        self.current_robot_id = 1
        self.current_goal = None
        self.start_time = 0
        self.state = 'INIT' # INIT, MOVING, DONE
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Sequential Nav Test Started')

    def load_vertices(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        vertices = data['levels']['L1']['vertices']
        walls = data['levels']['L1']['walls']
        
        wall_indices = set()
        for w in walls:
            wall_indices.add(w[0])
            wall_indices.add(w[1])
            
        self.vertices = vertices # Store full list
        self.walls = [] # Store actual coords
        
        # Parse walls
        for w in walls:
            idx1 = w[0]
            idx2 = w[1]
            p1 = vertices[idx1]
            p2 = vertices[idx2]
            self.walls.append(((p1[0], p1[1]), (p2[0], p2[1])))

        self.valid_vertices = []
        for i, v in enumerate(vertices):
            if i not in wall_indices:
                self.valid_vertices.append((float(v[0]), float(v[1])))
                
        self.get_logger().info(f"Loaded {len(self.valid_vertices)} valid vertices for goals")

    def collision_callback(self, msg, robot_id):
        # self.get_logger().info(f"Got collision msg for {robot_id}: {msg.data}")
        if msg.data and robot_id == self.current_robot_id and self.state == 'MOVING':
            self.get_logger().warn(f"Collision detected for Robot {robot_id}! Stopping.")
            self.send_vel(robot_id, 0.0, 0.0)
            self.current_robot_id += 1
            self.state = 'INIT'

    def pose_callback(self, msg, robot_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        self.poses[robot_id] = (x, y, theta)

    def check_line_of_sight(self, p1, p2):
        # p1, p2 are (x, y) tuples
        for w1, w2 in self.walls:
            if self.segments_intersect(p1, p2, w1, w2):
                return False
        return True

    def segments_intersect(self, p1, p2, p3, p4):
        # Standard line segment intersection
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
            
        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def control_loop(self):
        if self.current_robot_id > self.num_robots:
            self.get_logger().info("All robots finished!")
            # Graceful exit of test?
            rclpy.shutdown()
            return
            
        if self.state == 'INIT':
            # Check if we have pose
            if self.current_robot_id in self.poses:
                # Pick goal
                start_x, start_y, _ = self.poses[self.current_robot_id]
                start_pt = (start_x, start_y)
                
                # Pick goal: Sort vertices by distance DESC, pick first valid line of sight
                # This finds "other end of map" candidates
                
                # Sort valid_vertices by distance to start
                sorted_vertices = sorted(self.valid_vertices, key=lambda v: math.hypot(v[0]-start_x, v[1]-start_y), reverse=True)
                
                found_goal = False
                
                # Try top 20% farthest points or just iterate all
                for gx, gy in sorted_vertices:
                    goal_pt = (gx, gy)
                    # Check Line of Sight
                    if self.check_line_of_sight(start_pt, goal_pt):
                        self.current_goal = goal_pt
                        found_goal = True
                        break
                
                if not found_goal:
                    self.get_logger().warn(f"Robot {self.current_robot_id}: Could not find clear path goal. Skipping.")
                    self.current_robot_id += 1
                    return
                        
                self.get_logger().info(f"Robot {self.current_robot_id} starting -> {self.current_goal}")
                self.state = 'MOVING'
                self.start_time = time.time()
            return
            
        if self.state == 'MOVING':
             # Timeout check (e.g., 60 seconds)
             if time.time() - self.start_time > 60.0:
                 self.get_logger().warn(f"Robot {self.current_robot_id} timed out! Skipping.")
                 self.send_vel(self.current_robot_id, 0.0, 0.0)
                 self.current_robot_id += 1
                 self.state = 'INIT'
                 return

             if self.current_robot_id not in self.poses:
                 return # Lost pose?
                 
             rx, ry, rth = self.poses[self.current_robot_id]
             gx, gy = self.current_goal
             
             dist = math.hypot(gx - rx, gy - ry)
             
             if dist < 20:
                 self.get_logger().info(f"Robot {self.current_robot_id} reached goal.")
                 # Stop robot
                 self.send_vel(self.current_robot_id, 0.0, 0.0)
                 self.current_robot_id += 1
                 self.state = 'INIT'
                 return
                 
             # Heading logic (same as before)
             desired_th = math.atan2(gy - ry, gx - rx)
             angle_error = desired_th - rth
             while angle_error > math.pi: angle_error -= 2*math.pi
             while angle_error < -math.pi: angle_error += 2*math.pi
             
             v = 0.0
             w = 0.0
             
             if abs(angle_error) > 0.5:
                 w = 2.0 * angle_error
                 v = 0.0
             else:
                 w = 2.0 * angle_error
                 v = 100.0 # Speed
                 
             # Clamp
             v = max(min(v, 200.0), -200.0)
             w = max(min(w, 2.0), -2.0)
             
             self.send_vel(self.current_robot_id, v, w)

    def send_vel(self, robot_id, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pubs[robot_id].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SequentialNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
