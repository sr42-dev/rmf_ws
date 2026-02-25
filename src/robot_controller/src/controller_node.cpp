#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

#include <cmath>
#include <string>
#include <sstream>
#include <vector>

class RobotController : public rclcpp::Node
{
public:
  /**
   * Robot controller: receives PathRequest with a list of Location waypoints,
   * navigates through them sequentially using spin-in-place then drive-straight.
   * Publishes RobotState on robot_state topic.
   * Supports Location.t hold: if the next waypoint has t > 0, the robot waits
   * at the current waypoint until now() >= t before departing.
   * Supports preemption: a new PathRequest replaces the active path immediately.
   */
  explicit RobotController(int robot_id)
  : Node("robot_" + std::to_string(robot_id) + "_controller"),
    robot_id_(robot_id),
    robot_name_("robot_" + std::to_string(robot_id))
  {
    const std::string prefix = "/robot_" + std::to_string(robot_id);

    // Subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/pose", 10,
      std::bind(&RobotController::pose_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      prefix + "/scan", 10,
      std::bind(&RobotController::scan_callback, this, std::placeholders::_1));

    path_request_sub_ = this->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
      "robot_path_requests", 10,
      std::bind(&RobotController::path_request_callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      prefix + "/cmd_vel", 10);

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      prefix + "/status", 10);

    robot_state_pub_ = this->create_publisher<rmf_fleet_msgs::msg::RobotState>(
      "robot_state", 10);

    // Control loop timer (10 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotController::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
      "Robot %d controller started (PathRequest mode)", robot_id);
  }

private:
  // Phase constants
  enum Phase {
    PHASE_IDLE = 0,
    PHASE_SPIN = 1,
    PHASE_DRIVE = 2,
    PHASE_WAIT = 3
  };

  // Robot identity
  int robot_id_;
  std::string robot_name_;
  static constexpr const char* MODEL_NAME = "sim_robot";
  static constexpr const char* LEVEL_NAME = "L1";

  // Current state (from pose subscriber, in meters)
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_theta_ = 0.0;

  // Path state
  std::vector<rmf_fleet_msgs::msg::Location> path_;
  size_t path_index_ = 0;
  std::string task_id_;
  bool has_path_ = false;
  Phase phase_ = PHASE_IDLE;

  // Wait state (for Location.t hold)
  rclcpp::Time wait_until_{0, 0, RCL_ROS_TIME};

  // Scale factor: must match sim_node
  static constexpr double METER_TO_PIXEL = 30.0;

  // Controller parameters
  static constexpr double SPIN_VEL = 1.5;              // rad/s (max)
  static constexpr double SPIN_GAIN = 3.0;             // proportional gain for SPIN
  static constexpr double HEADING_TOLERANCE = 0.15;     // rad (~8.6°)
  static constexpr double DRIVE_VEL = 2.0;             // m/s
  static constexpr double GOAL_TOLERANCE = 0.5;         // meters
  static constexpr double SLOWDOWN_DISTANCE = 2.0;      // meters
  static constexpr double COURSE_CORRECT_GAIN = 2.0;    // angular correction gain during DRIVE
  static constexpr double HEADING_DRIFT_LIMIT = M_PI / 2.0; // 90° before back to SPIN

  int loop_count_ = 0;
  uint64_t fleet_state_seq_ = 0;
  uint64_t last_reached_index_ = 0;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr path_request_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Callbacks ──

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;
    current_theta_ = 2.0 * std::atan2(qz, qw);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/)
  {
    // LIDAR data not used for spin-then-drive
  }

  void path_request_callback(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
  {
    // Only accept requests for this robot
    if (msg->robot_name != robot_name_) {
      return;
    }

    // Store the path and task_id (preempts any active path)
    path_ = msg->path;
    task_id_ = msg->task_id;
    path_index_ = 0;
    has_path_ = !path_.empty();

    if (has_path_) {
      phase_ = PHASE_SPIN;
      RCLCPP_INFO(this->get_logger(),
        "PathRequest: task='%s', %zu waypoints → SPIN to wp 0 "
        "(first vertex=%lu, last vertex=%lu)",
        task_id_.c_str(), path_.size(),
        static_cast<unsigned long>(path_.front().index),
        static_cast<unsigned long>(path_.back().index));
    } else {
      phase_ = PHASE_IDLE;
      RCLCPP_WARN(this->get_logger(), "PathRequest with empty path");
    }
  }

  // ── Helpers ──

  static double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  double goal_x() const { return path_[path_index_].x; }
  double goal_y() const { return path_[path_index_].y; }

  double dist_to_goal() const
  {
    if (!has_path_ || path_index_ >= path_.size()) return 1e9;
    return std::hypot(goal_x() - current_x_, goal_y() - current_y_);
  }

  double heading_error() const
  {
    double dx = goal_x() - current_x_;
    double dy = goal_y() - current_y_;
    double goal_angle = std::atan2(dy, dx);
    return normalize_angle(goal_angle - current_theta_);
  }

  void publish_goal_reached()
  {
    double gx = goal_x();
    double gy = goal_y();
    std_msgs::msg::String status_msg;
    std::ostringstream oss;
    oss << "{\"robot_id\": " << robot_id_
        << ", \"status\": \"goal_reached\""
        << ", \"goal\": [" << gx << ", " << gy << "]"
        << ", \"waypoint_index\": " << path_index_
        << ", \"task_id\": \"" << task_id_ << "\"}";
    status_msg.data = oss.str();
    status_pub_->publish(status_msg);
  }

  void publish_robot_state()
  {
    rmf_fleet_msgs::msg::RobotState robot_state;
    robot_state.name = robot_name_;
    robot_state.model = MODEL_NAME;
    robot_state.task_id = task_id_;
    robot_state.seq = fleet_state_seq_++;
    robot_state.battery_percent = 100.0f;

    // Current location
    robot_state.location.x = static_cast<float>(current_x_);
    robot_state.location.y = static_cast<float>(current_y_);
    robot_state.location.yaw = static_cast<float>(current_theta_);
    robot_state.location.level_name = LEVEL_NAME;
    robot_state.location.index = last_reached_index_;
    robot_state.location.t = this->get_clock()->now();

    // Mode
    if (phase_ == PHASE_IDLE) {
      robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    } else if (phase_ == PHASE_WAIT) {
      robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_WAITING;
    } else {
      robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    }

    // Remaining path (from current target onward)
    if (has_path_ && path_index_ < path_.size()) {
      for (size_t i = path_index_; i < path_.size(); i++) {
        robot_state.path.push_back(path_[i]);
      }
    }

    robot_state_pub_->publish(robot_state);
  }

  /// Check if a Location.t is non-zero (specified departure time)
  static bool has_departure_time(const rmf_fleet_msgs::msg::Location & loc)
  {
    return loc.t.sec > 0 || loc.t.nanosec > 0;
  }

  /// Advance to the next waypoint. Returns true if there is one.
  bool advance_to_next_waypoint()
  {
    // Record vertex index of the waypoint we just reached
    last_reached_index_ = path_[path_index_].index;
    path_index_++;

    if (path_index_ >= path_.size()) {
      phase_ = PHASE_IDLE;
      has_path_ = false;
      RCLCPP_INFO(this->get_logger(),
        "All %zu waypoints reached for task '%s'! → IDLE",
        path_.size(), task_id_.c_str());
      return false;
    }

    // Check if next waypoint has a departure time (Location.t)
    if (has_departure_time(path_[path_index_])) {
      wait_until_ = rclcpp::Time(path_[path_index_].t);
      phase_ = PHASE_WAIT;
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu has departure t=%d.%09u → WAIT",
        path_index_,
        path_[path_index_].t.sec,
        path_[path_index_].t.nanosec);
      return true;
    }

    // No wait, go straight to SPIN for next waypoint
    phase_ = PHASE_SPIN;
    RCLCPP_INFO(this->get_logger(),
      "Advancing to wp %zu/%zu (%.2f, %.2f) vertex=%lu → SPIN",
      path_index_, path_.size() - 1, goal_x(), goal_y(),
      static_cast<unsigned long>(path_[path_index_].index));
    return true;
  }

  // ── Control loop ──

  void control_loop()
  {
    geometry_msgs::msg::Twist cmd;

    // Publish RobotState at 2 Hz (every 5th tick)
    loop_count_++;
    if (loop_count_ % 5 == 0) {
      publish_robot_state();
    }

    if (phase_ == PHASE_IDLE) {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    // ── WAIT phase: hold position until departure time ──
    if (phase_ == PHASE_WAIT) {
      cmd_vel_pub_->publish(cmd);  // zero velocity

      rclcpp::Time now = this->get_clock()->now();
      if (now >= wait_until_) {
        RCLCPP_INFO(this->get_logger(),
          "Wait done → SPIN for wp %zu", path_index_);
        phase_ = PHASE_SPIN;
      }
      return;
    }

    double dist = dist_to_goal();
    double herr = heading_error();

    // Waypoint reached?
    if (dist < GOAL_TOLERANCE) {
      cmd_vel_pub_->publish(cmd);  // stop
      publish_goal_reached();
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu reached! dist=%.3fm (vertex=%lu)",
        path_index_, dist,
        static_cast<unsigned long>(path_[path_index_].index));
      advance_to_next_waypoint();
      return;
    }

    // ── SPIN phase: proportional rotate in place toward goal ──
    if (phase_ == PHASE_SPIN) {
      if (std::abs(herr) < HEADING_TOLERANCE) {
        phase_ = PHASE_DRIVE;
        RCLCPP_INFO(this->get_logger(),
          "Heading aligned (%.1f°) → DRIVE", herr * 180.0 / M_PI);
        // Send zero velocity to stop spinning before driving
        cmd_vel_pub_->publish(cmd);
        return;
      } else {
        // Proportional control with saturation
        double angular = SPIN_GAIN * herr;
        if (angular > SPIN_VEL) angular = SPIN_VEL;
        if (angular < -SPIN_VEL) angular = -SPIN_VEL;
        cmd.angular.z = angular;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(),
          "cmd_vel SPIN: ang=%.3f vertex=%lu wp=%zu/%zu",
          angular,
          static_cast<unsigned long>(path_[path_index_].index),
          path_index_, path_.size() - 1);

        if (loop_count_ % 20 == 0) {
          RCLCPP_INFO(this->get_logger(),
            "SPIN  pos=(%.2f,%.2f) th=%.1f° herr=%.1f° dist=%.2f wp=%zu/%zu",
            current_x_, current_y_,
            current_theta_ * 180.0 / M_PI,
            herr * 180.0 / M_PI, dist,
            path_index_, path_.size() - 1);
        }
        return;
      }
    }

    // ── DRIVE phase: go straight, small course corrections ──
    if (phase_ == PHASE_DRIVE) {
      if (std::abs(herr) > HEADING_DRIFT_LIMIT) {
        phase_ = PHASE_SPIN;
        RCLCPP_INFO(this->get_logger(),
          "Heading drifted (%.1f°) → back to SPIN", herr * 180.0 / M_PI);
        cmd_vel_pub_->publish(cmd);
        return;
      }

      double linear = DRIVE_VEL;
      if (dist < SLOWDOWN_DISTANCE) {
        linear = DRIVE_VEL * (dist / SLOWDOWN_DISTANCE);
        if (linear < 0.3) linear = 0.3;
      }

      double angular = COURSE_CORRECT_GAIN * herr;

      cmd.linear.x = linear * METER_TO_PIXEL;  // px/s for sim
      cmd.angular.z = angular;
      cmd_vel_pub_->publish(cmd);

      RCLCPP_DEBUG(this->get_logger(),
        "cmd_vel DRIVE: lin=%.3f ang=%.3f vertex=%lu wp=%zu/%zu",
        linear, angular,
        static_cast<unsigned long>(path_[path_index_].index),
        path_index_, path_.size() - 1);

      if (loop_count_ % 20 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "DRIVE pos=(%.2f,%.2f) th=%.1f° herr=%.1f° dist=%.2f lin=%.2f wp=%zu/%zu",
          current_x_, current_y_,
          current_theta_ * 180.0 / M_PI,
          herr * 180.0 / M_PI, dist,
          linear,
          path_index_, path_.size() - 1);
      }
    }
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parse robot_id from arguments (e.g. robot_id:=2)
  int robot_id = 1;
  for (int i = 1; i < argc; i++) {
    std::string arg(argv[i]);
    if (arg.rfind("robot_id:=", 0) == 0) {
      robot_id = std::stoi(arg.substr(10));
    }
  }

  auto node = std::make_shared<RobotController>(robot_id);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
