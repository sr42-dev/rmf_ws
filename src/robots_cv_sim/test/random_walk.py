import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import random
import math
import sys

class RandomWalkController(Node):
    def __init__(self):
        super().__init__('random_walk_controller')
        
        # Determine number of robots
        # We can try to get it from a param or just look for topics.
        # But since we are running as a bare script, we can parse args or defaults.
        self.declare_parameter('num_robots', 100)
        # Allow override from ros args if passed
        self.num_robots = self.get_parameter('num_robots').value
        
        self.goals = {} # robot_id -> (x, y)
        self.poses = {} # robot_id -> (x, y, theta)
        
        self.pubs = []
        self.subs = []
        
        # Assume map bounds roughly 0-2000 for goals
        self.map_w = 2000
        self.map_h = 2000
        
        for i in range(self.num_robots):
            # Publisher
            pub = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10)
            self.pubs.append(pub)
            
            # Subscriber
            sub = self.create_subscription(
                PoseStamped, 
                f'/robot_{i}/pose',
                lambda msg, robot_id=i: self.pose_callback(msg, robot_id),
                10
            )
            self.subs.append(sub)
            
            # Init goal
            self.goals[i] = self.get_random_goal()
            
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Controller started')

    def get_random_goal(self):
        return (random.uniform(200, self.map_w-200), random.uniform(200, self.map_h-200))

    def pose_callback(self, msg, robot_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # Quaternion to theta
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        self.poses[robot_id] = (x, y, theta)

    def control_loop(self):
        for i in range(self.num_robots):
            if i not in self.poses:
                continue
                
            rx, ry, rth = self.poses[i]
            gx, gy = self.goals[i]
            
            # Distance
            dist = math.hypot(gx - rx, gy - ry)
            
            if dist < 20: # Reached goal
                self.goals[i] = self.get_random_goal()
                continue
            
            # Heading
            desired_th = math.atan2(gy - ry, gx - rx)
            angle_error = desired_th - rth
            
            # Normalize angle
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi
            
            msg = Twist()
            
            # Simple P controller
            if abs(angle_error) > 0.5:
                # Turn in place
                msg.angular.z = 2.0 * angle_error
                msg.linear.x = 0.0
            else:
                # Drive and turn
                msg.angular.z = 2.0 * angle_error
                msg.linear.x = 5.0 * 20.0 # 100 pixels/sec?
            
            # Clamp limits
            msg.linear.x = max(min(msg.linear.x, 200.0), -200.0)
            msg.angular.z = max(min(msg.angular.z, 2.0), -2.0)
            
            self.pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
