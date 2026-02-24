"""
Integration test that navigates a robot across the map using RMF PathRequest.
This test:
1. Launches the simulator and controller
2. Sends a single PathRequest with 11 waypoints to the robot
3. Monitors FleetState and status for waypoint completion
4. Verifies the robot reaches all waypoints without collision
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rmf_fleet_msgs.msg import PathRequest, RobotState, Location
import math
import time
import json
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import functools

# Force unbuffered print
print = functools.partial(print, flush=True)


# Scale factor: pixels to meters (must match sim_node)
PIXEL_TO_METER = 1.0 / 30.0


class NavigationTestNode(Node):
    """Test node that sends a PathRequest and monitors FleetState."""

    def __init__(self, robot_id: int = 1):
        super().__init__('navigation_test_node')

        self.robot_id = robot_id
        self.robot_name = f'robot_{robot_id}'

        # Current state
        self.current_x = None  # In meters
        self.current_y = None
        self.current_theta = None
        self.lidar_ranges = []
        self.in_collision = False
        self.collision_threshold = 1.0 * PIXEL_TO_METER  # meters (1 pixel)

        # FleetState tracking
        self.fleet_path_remaining = None  # number of waypoints left
        self.fleet_mode = None
        self.fleet_task_id = None

        # Goal-reached tracking (from JSON status topic)
        self.goal_reached = False
        self.last_status = None
        self.pending_goal_x = None
        self.pending_goal_y = None
        self.waypoints_reached_count = 0

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/robot_{robot_id}/pose',
            self.pose_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/robot_{robot_id}/scan',
            self.scan_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            f'/robot_{robot_id}/status',
            self.status_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_callback,
            10
        )

        # Publisher for PathRequest
        self.path_request_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            10
        )

        self.get_logger().info(
            f'Navigation test node started for {self.robot_name}')

    def pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def scan_callback(self, msg: LaserScan):
        self.lidar_ranges = list(msg.ranges)
        self.in_collision = any(
            r < self.collision_threshold for r in self.lidar_ranges)

    def status_callback(self, msg: String):
        self.last_status = msg.data
        try:
            status_data = json.loads(msg.data)
            if status_data.get('status') == 'goal_reached':
                g = status_data.get('goal', [None, None])
                if (self.pending_goal_x is not None
                        and abs(g[0] - self.pending_goal_x) < 0.1
                        and abs(g[1] - self.pending_goal_y) < 0.1):
                    self.goal_reached = True
                    self.waypoints_reached_count += 1
                    self.get_logger().info(
                        f'goal_reached for ({g[0]:.2f},{g[1]:.2f}) '
                        f'[wp {self.waypoints_reached_count}]')
        except (json.JSONDecodeError, TypeError, IndexError):
            pass

    def robot_state_callback(self, msg: RobotState):
        if msg.name == self.robot_name:
            self.fleet_path_remaining = len(msg.path)
            self.fleet_mode = msg.mode.mode
            self.fleet_task_id = msg.task_id

    def send_path_request(self, waypoints, task_id='nav_test_1'):
        """Send a PathRequest with the given waypoints.

        waypoints: list of (x_meters, y_meters, vertex_index) tuples
        """
        msg = PathRequest()
        msg.fleet_name = 'fleet_1'
        msg.robot_name = self.robot_name
        msg.task_id = task_id

        for wx, wy, vidx in waypoints:
            loc = Location()
            # t left at default (zero) — no timing constraints
            loc.x = float(wx)
            loc.y = float(wy)
            loc.yaw = 0.0
            loc.level_name = 'L1'
            loc.index = int(vidx)
            msg.path.append(loc)

        # Drain stale messages
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)

        self.path_request_pub.publish(msg)
        self.get_logger().info(
            f'Sent PathRequest: task={task_id}, {len(msg.path)} waypoints')

    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        """Wait until we receive a pose."""
        start_time = time.time()
        while self.current_x is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_x is not None

    def wait_for_waypoint(self, goal_x, goal_y, timeout=60.0):
        """Wait for robot to reach a specific waypoint.

        Returns (success, collision_occurred).
        """
        self.goal_reached = False
        self.pending_goal_x = goal_x
        self.pending_goal_y = goal_y

        start_time = time.time()
        collision_occurred = False
        last_log = start_time

        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.in_collision:
                collision_occurred = True
                self.get_logger().warn('Collision detected!')

            if self.goal_reached:
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f'Waypoint reached after {elapsed:.1f}s')
                return True, collision_occurred

            now = time.time()
            if now - last_log >= 5.0:
                elapsed = now - start_time
                pos = (f'({self.current_x:.2f}, {self.current_y:.2f})'
                       if self.current_x else '(?,?)')
                remaining = (self.fleet_path_remaining
                             if self.fleet_path_remaining is not None
                             else '?')
                self.get_logger().info(
                    f'Waiting... {elapsed:.0f}s, pos={pos}, '
                    f'path_remaining={remaining}')
                last_log = now

        elapsed = time.time() - start_time
        self.get_logger().info(f'Waypoint TIMEOUT after {elapsed:.1f}s')
        return False, collision_occurred

    def get_distance_to(self, goal_x, goal_y):
        if self.current_x is None or self.current_y is None:
            return float('inf')
        return math.hypot(goal_x - self.current_x, goal_y - self.current_y)


def load_waypoints_from_map():
    """Load waypoints by reading specific vertex indices from the
    building YAML in traffic_editor_assets.

    Returns list of (x_meters, y_meters, vertex_index) tuples.
    """
    VERTEX_INDICES = [
        4860, 4873, 5373, 5402, 5697, 5762,
        7049, 7058, 9468, 9363, 6474, 6476,
        5148, 5136, 4861,
    ]

    assets_dir = get_package_share_directory('traffic_editor_assets')
    building_yaml = os.path.join(assets_dir, 'map.building.yaml')

    with open(building_yaml, 'r') as f:
        data = yaml.safe_load(f)

    vertices = data['levels']['L1']['vertices']

    waypoints = []
    for idx in VERTEX_INDICES:
        if idx >= len(vertices):
            print(f"WARNING: vertex index {idx} out of range "
                  f"(max {len(vertices)-1}), skipping")
            continue
        v = vertices[idx]
        waypoints.append(
            (v[0] * PIXEL_TO_METER, v[1] * PIXEL_TO_METER, idx))

    if not waypoints:
        print("WARNING: No waypoints loaded, using fallback")
        return [(13.33, 13.33, 0), (26.67, 13.33, 0),
                (40.0, 22.83, 0), (26.67, 22.83, 0)]

    return waypoints


def run_navigation_test():
    """Run the navigation test across the map."""
    rclpy.init()

    test_node = NavigationTestNode(robot_id=1)

    # Wait for simulator to be ready
    print("Waiting for simulator...")
    if not test_node.wait_for_pose(timeout=10.0):
        print("FAILED: Could not receive robot pose. "
              "Is the simulator running?")
        test_node.destroy_node()
        rclpy.shutdown()
        return False

    print(f"Robot initial position: "
          f"({test_node.current_x:.2f}, {test_node.current_y:.2f}) meters")

    # Load waypoints from map building YAML
    waypoints = load_waypoints_from_map()
    print(f"Loaded {len(waypoints)} waypoints from map vertices:")
    for j, (wx, wy, vidx) in enumerate(waypoints):
        print(f"  WP{j+1}: ({wx:.2f}, {wy:.2f}) m  "
              f"[px: ({wx/PIXEL_TO_METER:.0f}, {wy/PIXEL_TO_METER:.0f})] "
              f"vertex={vidx}")

    # Send ALL waypoints as a single PathRequest
    test_node.send_path_request(waypoints, task_id='nav_test_1')

    test_results = []
    total_collisions = 0

    print("\n" + "=" * 60)
    print("STARTING NAVIGATION TEST")
    print("=" * 60)

    # Monitor each waypoint being reached sequentially
    for i, (goal_x, goal_y, vidx) in enumerate(waypoints):
        print(f"\n--- Waypoint {i+1}/{len(waypoints)}: "
              f"({goal_x:.1f}, {goal_y:.1f}) m  vertex={vidx} ---")

        success, collision = test_node.wait_for_waypoint(
            goal_x, goal_y, timeout=120.0)

        if collision:
            total_collisions += 1

        final_dist = test_node.get_distance_to(goal_x, goal_y)

        result = {
            'waypoint': i + 1,
            'goal': (goal_x, goal_y),
            'vertex_index': vidx,
            'reached': success,
            'collision': collision,
            'final_distance': final_dist
        }
        test_results.append(result)

        if success:
            print(f"✓ Waypoint {i+1} REACHED "
                  f"(distance: {final_dist:.2f}m)")
        else:
            print(f"✗ Waypoint {i+1} FAILED - timeout "
                  f"(distance: {final_dist:.2f}m)")

        if collision:
            print("  ⚠ Collision occurred during navigation!")

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    waypoints_reached = sum(1 for r in test_results if r['reached'])
    print(f"Waypoints reached: {waypoints_reached}/{len(waypoints)}")
    print(f"Total collisions: {total_collisions}")

    all_passed = (waypoints_reached == len(waypoints)
                  and total_collisions == 0)

    if all_passed:
        print("\n✓ ALL TESTS PASSED")
    else:
        print("\n✗ SOME TESTS FAILED")

    # Cleanup
    test_node.destroy_node()
    rclpy.shutdown()

    return all_passed


def main():
    success = run_navigation_test()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
