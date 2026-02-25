"""
Integration test that issues a patrol task to the fleet adapter and verifies
the robot traverses all checkpoints.

This test:
1. Publishes an ApiRequest (patrol type) to /task_api_requests
2. Monitors /robot_1/status for goal_reached JSON events (same verification
   method as test_navigation.py)
3. Subscribes to /robot_1/cmd_vel to confirm the robot is actively moving
4. Verifies all checkpoints are reached in order

The checkpoint list is loaded from config/test_patrol.yaml.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rmf_task_msgs.msg import ApiRequest, ApiResponse
import math
import time
import json
import sys
import yaml
import uuid
from ament_index_python.packages import get_package_share_directory
import os
import functools

# Force unbuffered print
print = functools.partial(print, flush=True)

# Scale factor: pixels to meters (must match sim_node)
PIXEL_TO_METER = 1.0 / 30.0


class PatrolTestNode(Node):
    """Test node that issues a patrol task via ApiRequest and monitors
    checkpoint completion through the robot status topic."""

    def __init__(self, robot_id: int = 1):
        super().__init__('patrol_test_node')

        self.robot_id = robot_id
        self.robot_name = f'robot_{robot_id}'

        # Goal-reached tracking (from JSON status topic)
        self.goal_reached = False
        self.pending_goal_x = None
        self.pending_goal_y = None
        self.waypoints_reached_count = 0

        # cmd_vel tracking — confirms robot is moving
        self.cmd_vel_received = False
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.motion_detected_between_checkpoints = False

        # Task response tracking
        self.task_response_received = False
        self.task_response_success = False
        self.task_response_message = ''

        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            f'/robot_{robot_id}/status',
            self.status_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/robot_{robot_id}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.task_response_sub = self.create_subscription(
            ApiResponse,
            '/task_api_responses',
            self.task_response_callback,
            10
        )

        # Publisher for task requests
        self.task_request_pub = self.create_publisher(
            ApiRequest,
            '/task_api_requests',
            10
        )

        self.get_logger().info(
            f'Patrol test node started for {self.robot_name}')

    def status_callback(self, msg: String):
        """Parse goal_reached events from the robot status topic.
        Same verification logic as test_navigation.py."""
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
                        f'[checkpoint {self.waypoints_reached_count}]')
        except (json.JSONDecodeError, TypeError, IndexError):
            pass

    def cmd_vel_callback(self, msg: Twist):
        """Track velocity commands to confirm robot is moving."""
        self.cmd_vel_received = True
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z

        # Detect non-trivial motion
        if abs(msg.linear.x) > 0.1 or abs(msg.angular.z) > 0.05:
            self.motion_detected_between_checkpoints = True

    def task_response_callback(self, msg: ApiResponse):
        """Handle ApiResponse from fleet adapter."""
        self.task_response_received = True
        try:
            resp_data = json.loads(msg.json_msg)
            self.task_response_success = resp_data.get('success', False)
            self.task_response_message = resp_data.get('message', '')
        except (json.JSONDecodeError, TypeError):
            self.task_response_success = False
            self.task_response_message = msg.json_msg

        self.get_logger().info(
            f'Task response: success={self.task_response_success}, '
            f'message={self.task_response_message}')

    def send_patrol_task(self, checkpoint_vertices, fleet_name='fleet_1'):
        """Publish a patrol task ApiRequest."""
        request_id = f'patrol_{uuid.uuid4().hex[:8]}'

        task_json = json.dumps({
            'type': 'patrol',
            'robot_name': self.robot_name,
            'fleet_name': fleet_name,
            'checkpoint_vertices': checkpoint_vertices,
        })

        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = task_json

        # Drain stale messages
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.01)

        self.task_request_pub.publish(msg)
        self.get_logger().info(
            f'Sent patrol task: id={request_id}, '
            f'{len(checkpoint_vertices)} checkpoints')

        return request_id

    def wait_for_task_response(self, timeout: float = 10.0) -> bool:
        """Wait for the fleet adapter to acknowledge the task."""
        start_time = time.time()
        while not self.task_response_received and \
                (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.task_response_received and self.task_response_success

    def wait_for_checkpoint(self, goal_x, goal_y, timeout=120.0):
        """Wait for robot to reach a specific checkpoint via status topic.

        Returns (success, had_motion).
        """
        self.goal_reached = False
        self.pending_goal_x = goal_x
        self.pending_goal_y = goal_y
        self.motion_detected_between_checkpoints = False

        start_time = time.time()
        last_log = start_time

        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.goal_reached:
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f'Checkpoint reached after {elapsed:.1f}s')
                return True, self.motion_detected_between_checkpoints

            now = time.time()
            if now - last_log >= 5.0:
                elapsed = now - start_time
                cmd_info = (f'lin={self.last_cmd_linear:.2f}, '
                            f'ang={self.last_cmd_angular:.2f}')
                self.get_logger().info(
                    f'Waiting... {elapsed:.0f}s, '
                    f'cmd_vel: {cmd_info}, '
                    f'motion_detected: '
                    f'{self.motion_detected_between_checkpoints}')
                last_log = now

        elapsed = time.time() - start_time
        self.get_logger().info(f'Checkpoint TIMEOUT after {elapsed:.1f}s')
        return False, self.motion_detected_between_checkpoints


def load_test_config():
    """Load test configuration from config/test_patrol.yaml.

    Returns (num_robots, checkpoint_vertices).
    """
    tests_dir = get_package_share_directory('rmf_tests')
    config_path = os.path.join(tests_dir, 'config', 'test_patrol.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    num_robots = config.get('num_robots', 1)
    checkpoints = config.get('checkpoints', [])
    return num_robots, checkpoints


def resolve_checkpoint_coordinates(checkpoint_vertices):
    """Resolve vertex indices to (x_m, y_m) using the building YAML.

    Returns list of (x_meters, y_meters, vertex_index) tuples.
    """
    assets_dir = get_package_share_directory('traffic_editor_assets')
    building_yaml = os.path.join(assets_dir, 'map.building.yaml')

    with open(building_yaml, 'r') as f:
        data = yaml.safe_load(f)

    vertices = data['levels']['L1']['vertices']
    waypoints = []

    for idx in checkpoint_vertices:
        if idx >= len(vertices):
            print(f"WARNING: vertex index {idx} out of range "
                  f"(max {len(vertices)-1}), skipping")
            continue
        v = vertices[idx]
        waypoints.append(
            (v[0] * PIXEL_TO_METER, v[1] * PIXEL_TO_METER, idx))

    return waypoints


def run_patrol_test():
    """Run the patrol task test."""
    rclpy.init()

    # Load config
    num_robots, checkpoint_vertices = load_test_config()
    print(f"Test config: num_robots={num_robots}, "
          f"{len(checkpoint_vertices)} checkpoints")

    if not checkpoint_vertices:
        print("FAILED: No checkpoints defined in test_patrol.yaml")
        rclpy.shutdown()
        return False

    # Resolve checkpoint coordinates for verification
    waypoints = resolve_checkpoint_coordinates(checkpoint_vertices)
    if not waypoints:
        print("FAILED: Could not resolve any checkpoint coordinates")
        rclpy.shutdown()
        return False

    print(f"Resolved {len(waypoints)} checkpoints:")
    for j, (wx, wy, vidx) in enumerate(waypoints):
        print(f"  CP{j+1}: ({wx:.2f}, {wy:.2f}) m  vertex={vidx}")

    test_node = PatrolTestNode(robot_id=1)

    # Wait a moment for subscribers to connect
    print("Waiting for fleet adapter connection...")
    start_wait = time.time()
    while (time.time() - start_wait) < 3.0:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Send the patrol task
    print("\nSending patrol task to fleet adapter...")
    request_id = test_node.send_patrol_task(
        checkpoint_vertices, fleet_name='fleet_1')

    # Wait for task acknowledgment
    if not test_node.wait_for_task_response(timeout=10.0):
        print("FAILED: Fleet adapter did not acknowledge patrol task")
        test_node.destroy_node()
        rclpy.shutdown()
        return False

    print("Fleet adapter acknowledged patrol task")

    # Monitor each checkpoint
    test_results = []

    print("\n" + "=" * 60)
    print("STARTING PATROL TEST")
    print("=" * 60)

    for i, (goal_x, goal_y, vidx) in enumerate(waypoints):
        print(f"\n--- Checkpoint {i+1}/{len(waypoints)}: "
              f"({goal_x:.1f}, {goal_y:.1f}) m  vertex={vidx} ---")

        success, had_motion = test_node.wait_for_checkpoint(
            goal_x, goal_y, timeout=120.0)

        result = {
            'checkpoint': i + 1,
            'goal': (goal_x, goal_y),
            'vertex_index': vidx,
            'reached': success,
            'had_motion': had_motion,
        }
        test_results.append(result)

        if success:
            motion_str = "✓ motion" if had_motion else "⚠ no motion detected"
            print(f"✓ Checkpoint {i+1} REACHED ({motion_str})")
        else:
            print(f"✗ Checkpoint {i+1} FAILED - timeout")

    # Summary
    print("\n" + "=" * 60)
    print("PATROL TEST SUMMARY")
    print("=" * 60)

    checkpoints_reached = sum(1 for r in test_results if r['reached'])
    checkpoints_with_motion = sum(
        1 for r in test_results if r['had_motion'])

    print(f"Checkpoints reached: {checkpoints_reached}/{len(waypoints)}")
    print(f"Checkpoints with motion: {checkpoints_with_motion}/{len(waypoints)}")

    all_passed = (checkpoints_reached == len(waypoints))

    if all_passed:
        print("\n✓ ALL CHECKPOINTS PASSED")
    else:
        print("\n✗ SOME CHECKPOINTS FAILED")

    # Cleanup
    test_node.destroy_node()
    rclpy.shutdown()

    return all_passed


def main():
    success = run_patrol_test()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
