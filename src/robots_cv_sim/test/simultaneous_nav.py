import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import random
import math
import yaml
import sys
import time

class SimultaneousNav(Node):
    def __init__(self):
        super().__init__('simultaneous_nav')
        
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # Load map to get vertices for goals
        self.load_vertices('/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.building.yaml')
        
        self.pubs = {}
        self.subs = {}
        self.collision_subs = {}
        self.poses = {}
        self.goals = {}
        self.reached = {}
        
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
            
            self.reached[i] = False
            
        self.state = 'INIT' # INIT, MOVING, DONE
        self.start_time = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Simultaneous Nav Test Started')

    def load_vertices(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        vertices = data['levels']['L1']['vertices']
        walls = data['levels']['L1']['walls']
        
        wall_indices = set()
        for w in walls:
            wall_indices.add(w[0])
            wall_indices.add(w[1])
            
        self.vertices = vertices
        self.walls = []
        
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
        if msg.data:
            self.get_logger().warn(f"Collision detected for Robot {robot_id}!")

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

    def pick_farthest_goal(self, start_x, start_y):
        """Pick the farthest valid goal with clear line of sight"""
        start_pt = (start_x, start_y)
        
        # Sort valid_vertices by distance DESC
        sorted_vertices = sorted(self.valid_vertices, 
                                key=lambda v: math.hypot(v[0]-start_x, v[1]-start_y), 
                                reverse=True)
        
        # Try top farthest points
        for gx, gy in sorted_vertices:
            goal_pt = (gx, gy)
            if self.check_line_of_sight(start_pt, goal_pt):
                return goal_pt
        
        return None

    def control_loop(self):
        if self.state == 'INIT':
            # Debug
            if int(time.time() * 10) % 20 == 0: # Print every 2s
                self.get_logger().info(f"Waiting for poses... Have {len(self.poses)}/{self.num_robots}")

            # Wait for all poses
            if len(self.poses) == self.num_robots:
                # Pick goals for all robots
                for robot_id in range(1, self.num_robots + 1):
                    start_x, start_y, _ = self.poses[robot_id]
                    goal = self.pick_farthest_goal(start_x, start_y)
                    
                    if goal:
                        self.goals[robot_id] = goal
                        self.get_logger().info(f"Robot {robot_id} goal: {goal}")
                    else:
                        self.get_logger().warn(f"Robot {robot_id}: Could not find clear path goal!")
                        self.reached[robot_id] = True  # Skip this robot
                
                self.state = 'MOVING'
                self.start_time = time.time()
                self.get_logger().info("All goals assigned. Starting simultaneous navigation!")
            return
            
        if self.state == 'MOVING':
            # Check if all done
            if all(self.reached.values()):
                self.get_logger().info("All robots reached their goals!")
                for i in range(1, self.num_robots + 1):
                    self.send_vel(i, 0.0, 0.0)
                rclpy.shutdown()
                return
            
            # Timeout check (e.g., 120 seconds for all)
            if time.time() - self.start_time > 120.0:
                self.get_logger().warn("Timeout! Stopping all robots.")
                for i in range(1, self.num_robots + 1):
                    self.send_vel(i, 0.0, 0.0)
                rclpy.shutdown()
                return
            
            # Control each robot independently
            for robot_id in range(1, self.num_robots + 1):
                if self.reached[robot_id]:
                    continue
                    
                if robot_id not in self.poses or robot_id not in self.goals:
                    continue
                
                rx, ry, rth = self.poses[robot_id]
                gx, gy = self.goals[robot_id]
                
                dist = math.hypot(gx - rx, gy - ry)
                
                if dist < 20:
                    self.get_logger().info(f"Robot {robot_id} reached goal!")
                    self.send_vel(robot_id, 0.0, 0.0)
                    self.reached[robot_id] = True
                    continue
                
                # Simple navigation with robot-robot collision avoidance
                desired_th = math.atan2(gy - ry, gx - rx)
                angle_error = desired_th - rth
                while angle_error > math.pi: angle_error -= 2*math.pi
                while angle_error < -math.pi: angle_error += 2*math.pi
                
                v = 0.0
                w = 0.0
                
                # Check proximity to other robots
                too_close = False
                repulsion_x = 0.0
                repulsion_y = 0.0
                
                for other_id in range(1, self.num_robots + 1):
                    if other_id == robot_id or other_id not in self.poses:
                        continue
                    
                    ox, oy, _ = self.poses[other_id]
                    dist_to_other = math.hypot(ox - rx, oy - ry)
                    
                    if dist_to_other < 30:  # Safety distance
                        too_close = True
                        # Add repulsion vector
                        if dist_to_other > 0:
                            repulsion_x += (rx - ox) / dist_to_other
                            repulsion_y += (ry - oy) / dist_to_other
                
                if too_close:
                    # Slow down or steer away
                    if abs(repulsion_x) > 0.1 or abs(repulsion_y) > 0.1:
                        # Adjust heading based on repulsion
                        repulsion_angle = math.atan2(repulsion_y, repulsion_x)
                        angle_error = repulsion_angle - rth
                        while angle_error > math.pi: angle_error -= 2*math.pi
                        while angle_error < -math.pi: angle_error += 2*math.pi
                        
                        w = 3.0 * angle_error
                        v = 30.0  # Slow speed
                    else:
                        # Stop if very close
                        v = 0.0
                        w = 0.0
                else:
                    # Normal navigation
                    if abs(angle_error) > 0.5:
                        w = 2.0 * angle_error
                        v = 0.0
                    else:
                        w = 2.0 * angle_error
                        v = 100.0
                
                # Clamp
                v = max(min(v, 200.0), -200.0)
                w = max(min(w, 2.0), -2.0)
                
                self.send_vel(robot_id, v, w)

    def send_vel(self, robot_id, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pubs[robot_id].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimultaneousNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
