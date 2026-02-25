/**
 * Fleet Adapter Node
 *
 * Bridges high-level task requests to low-level robot PathRequests.
 *
 * Responsibilities:
 *  1. Subscribe to /task_api_requests (rmf_task_msgs/ApiRequest) — JSON payload
 *     with a "type" discriminator. Currently handles "patrol" tasks; extensible
 *     to delivery, clean, etc. by adding new handler methods.
 *  2. Convert patrol checkpoint vertex IDs → Location waypoints using the
 *     nav graph (vertex index → meter coordinates).
 *  3. Publish PathRequest on robot_path_requests for the target robot.
 *  4. Aggregate individual RobotState messages into a combined FleetState
 *     published on /fleet_state at 2 Hz.
 *  5. Respond on /task_api_responses with success/error.
 *
 * Parameters:
 *   nav_graph_path (string) — path to the generated nav graph YAML
 *                              default: /tmp/rmf_nav_graph.yaml
 *   fleet_name     (string) — name of the fleet, default: fleet_1
 */

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <chrono>
#include <mutex>

// Minimal JSON helpers — avoids pulling in a full JSON library.
// These are intentionally simple; they handle the flat schemas we produce.
namespace json_helpers
{

/// Extract a string value for a given key from a JSON string.
/// Returns empty string if not found.
inline std::string get_string(const std::string & json, const std::string & key)
{
  std::string search = "\"" + key + "\"";
  auto pos = json.find(search);
  if (pos == std::string::npos) return "";
  pos = json.find(':', pos + search.size());
  if (pos == std::string::npos) return "";
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) return "";
  auto end = json.find('"', pos + 1);
  if (end == std::string::npos) return "";
  return json.substr(pos + 1, end - pos - 1);
}

/// Extract an array of uint64 values for a given key from a JSON string.
inline std::vector<uint64_t> get_uint64_array(
  const std::string & json, const std::string & key)
{
  std::vector<uint64_t> result;
  std::string search = "\"" + key + "\"";
  auto pos = json.find(search);
  if (pos == std::string::npos) return result;
  pos = json.find('[', pos + search.size());
  if (pos == std::string::npos) return result;
  auto end = json.find(']', pos + 1);
  if (end == std::string::npos) return result;

  std::string arr = json.substr(pos + 1, end - pos - 1);
  std::istringstream iss(arr);
  std::string token;
  while (std::getline(iss, token, ',')) {
    // Trim whitespace
    auto start = token.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) continue;
    token = token.substr(start);
    auto stop = token.find_last_not_of(" \t\n\r");
    if (stop != std::string::npos) token = token.substr(0, stop + 1);
    if (!token.empty()) {
      result.push_back(static_cast<uint64_t>(std::stoull(token)));
    }
  }
  return result;
}

/// Build a simple JSON response string.
inline std::string make_response_json(
  bool success, const std::string & message, const std::string & task_id)
{
  std::ostringstream oss;
  oss << "{\"success\": " << (success ? "true" : "false")
      << ", \"message\": \"" << message << "\""
      << ", \"task_id\": \"" << task_id << "\"}";
  return oss.str();
}

}  // namespace json_helpers


class FleetAdapterNode : public rclcpp::Node
{
public:
  FleetAdapterNode()
  : Node("fleet_adapter")
  {
    // ── Parameters ──
    this->declare_parameter<std::string>(
      "nav_graph_path", "/tmp/rmf_nav_graph.yaml");
    this->declare_parameter<std::string>("fleet_name", "fleet_1");

    nav_graph_path_ = this->get_parameter("nav_graph_path")
      .as_string();
    fleet_name_ = this->get_parameter("fleet_name").as_string();

    // ── Load nav graph ──
    load_nav_graph(nav_graph_path_);

    // ── Subscribers ──
    task_request_sub_ = this->create_subscription<rmf_task_msgs::msg::ApiRequest>(
      "/task_api_requests", 10,
      std::bind(&FleetAdapterNode::task_request_callback, this,
        std::placeholders::_1));

    robot_state_sub_ = this->create_subscription<rmf_fleet_msgs::msg::RobotState>(
      "robot_state", 10,
      std::bind(&FleetAdapterNode::robot_state_callback, this,
        std::placeholders::_1));

    // ── Publishers ──
    path_request_pub_ = this->create_publisher<rmf_fleet_msgs::msg::PathRequest>(
      "robot_path_requests", 10);

    task_response_pub_ = this->create_publisher<rmf_task_msgs::msg::ApiResponse>(
      "/task_api_responses", 10);

    fleet_state_pub_ = this->create_publisher<rmf_fleet_msgs::msg::FleetState>(
      "/fleet_state", 10);

    // ── Fleet state timer (2 Hz) ──
    fleet_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FleetAdapterNode::publish_fleet_state, this));

    RCLCPP_INFO(this->get_logger(),
      "Fleet adapter started — fleet='%s', nav_graph='%s', %zu vertices loaded",
      fleet_name_.c_str(), nav_graph_path_.c_str(), vertex_map_.size());
  }

private:
  // ── Types ──
  struct VertexCoord {
    double x;  // meters
    double y;  // meters
  };

  // ── Data ──
  std::string nav_graph_path_;
  std::string fleet_name_;

  /// Vertex index → (x_m, y_m)
  std::unordered_map<uint64_t, VertexCoord> vertex_map_;

  /// Robot name → latest RobotState
  std::map<std::string, rmf_fleet_msgs::msg::RobotState> robot_states_;
  std::mutex robot_states_mutex_;

  uint64_t task_counter_ = 0;

  // ── ROS interfaces ──
  rclcpp::Subscription<rmf_task_msgs::msg::ApiRequest>::SharedPtr task_request_sub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr path_request_pub_;
  rclcpp::Publisher<rmf_task_msgs::msg::ApiResponse>::SharedPtr task_response_pub_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_pub_;
  rclcpp::TimerBase::SharedPtr fleet_state_timer_;

  // ═══════════════════════════════════════════════════════════════════
  //  Nav graph loading
  // ═══════════════════════════════════════════════════════════════════

  void load_nav_graph(const std::string & path)
  {
    RCLCPP_INFO(this->get_logger(), "Loading nav graph from: %s", path.c_str());

    YAML::Node graph;
    try {
      graph = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to load nav graph YAML: %s", e.what());
      return;
    }

    // Format: levels → L1 → vertices: list of [x_m, y_m, {options}]
    auto levels = graph["levels"];
    if (!levels || !levels.IsMap()) {
      RCLCPP_ERROR(this->get_logger(), "Nav graph missing 'levels' key");
      return;
    }

    for (auto level_it = levels.begin(); level_it != levels.end(); ++level_it) {
      std::string level_name = level_it->first.as<std::string>();
      auto vertices = level_it->second["vertices"];
      if (!vertices || !vertices.IsSequence()) {
        RCLCPP_WARN(this->get_logger(),
          "Level '%s' has no vertices list", level_name.c_str());
        continue;
      }

      for (size_t i = 0; i < vertices.size(); ++i) {
        auto v = vertices[i];
        if (!v.IsSequence() || v.size() < 2) continue;

        VertexCoord coord;
        coord.x = v[0].as<double>();
        coord.y = v[1].as<double>();
        vertex_map_[static_cast<uint64_t>(i)] = coord;
      }

      RCLCPP_INFO(this->get_logger(),
        "Level '%s': loaded %zu vertices", level_name.c_str(), vertices.size());
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  //  Task request handling
  // ═══════════════════════════════════════════════════════════════════

  void task_request_callback(
    const rmf_task_msgs::msg::ApiRequest::SharedPtr msg)
  {
    const std::string & json = msg->json_msg;
    const std::string & request_id = msg->request_id;

    RCLCPP_INFO(this->get_logger(),
      "Received task request [id=%s]: %s",
      request_id.c_str(), json.c_str());

    std::string task_type = json_helpers::get_string(json, "type");

    if (task_type == "patrol") {
      handle_patrol_task(json, request_id);
    } else {
      std::string err = "Unknown task type: '" + task_type + "'";
      RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
      send_response(request_id, false, err);
    }
  }

  // ── Patrol task handler ──

  void handle_patrol_task(
    const std::string & json, const std::string & request_id)
  {
    std::string robot_name = json_helpers::get_string(json, "robot_name");
    std::string task_fleet = json_helpers::get_string(json, "fleet_name");
    auto checkpoints = json_helpers::get_uint64_array(json, "checkpoint_vertices");

    if (robot_name.empty()) {
      send_response(request_id, false, "Missing 'robot_name' in patrol task");
      return;
    }
    if (checkpoints.empty()) {
      send_response(request_id, false,
        "Missing or empty 'checkpoint_vertices' in patrol task");
      return;
    }
    if (task_fleet.empty()) {
      task_fleet = fleet_name_;
    }

    RCLCPP_INFO(this->get_logger(),
      "Patrol task: robot='%s', fleet='%s', %zu checkpoints",
      robot_name.c_str(), task_fleet.c_str(), checkpoints.size());

    // Resolve vertex IDs to Location waypoints
    std::vector<rmf_fleet_msgs::msg::Location> path;
    for (size_t i = 0; i < checkpoints.size(); ++i) {
      uint64_t vid = checkpoints[i];
      auto it = vertex_map_.find(vid);
      if (it == vertex_map_.end()) {
        std::string err = "Vertex " + std::to_string(vid) + " not found in nav graph";
        RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
        send_response(request_id, false, err);
        return;
      }

      rmf_fleet_msgs::msg::Location loc;
      loc.x = static_cast<float>(it->second.x);
      loc.y = static_cast<float>(it->second.y);
      loc.yaw = 0.0f;
      loc.level_name = "L1";
      loc.index = vid;
      // t left at default (zero) — no timing constraints
      path.push_back(loc);

      RCLCPP_INFO(this->get_logger(),
        "  Checkpoint %zu: vertex %lu → (%.2f, %.2f) m",
        i + 1, vid, it->second.x, it->second.y);
    }

    // Build and publish PathRequest
    rmf_fleet_msgs::msg::PathRequest path_req;
    path_req.fleet_name = task_fleet;
    path_req.robot_name = robot_name;
    path_req.task_id = request_id;
    path_req.path = path;

    path_request_pub_->publish(path_req);

    RCLCPP_INFO(this->get_logger(),
      "Published PathRequest: task='%s', robot='%s', %zu waypoints "
      "(first vertex=%lu, last vertex=%lu)",
      request_id.c_str(), robot_name.c_str(), path.size(),
      checkpoints.front(), checkpoints.back());

    send_response(request_id, true,
      "Patrol task dispatched with " + std::to_string(path.size()) + " waypoints");
  }

  // ── Future task handlers would go here ──
  // void handle_delivery_task(const std::string & json, const std::string & request_id);
  // void handle_clean_task(const std::string & json, const std::string & request_id);

  // ── Response helper ──

  void send_response(
    const std::string & request_id, bool success, const std::string & message)
  {
    rmf_task_msgs::msg::ApiResponse resp;
    resp.type = rmf_task_msgs::msg::ApiResponse::TYPE_RESPONDING;
    resp.json_msg = json_helpers::make_response_json(
      success, message, request_id);
    resp.request_id = request_id;
    task_response_pub_->publish(resp);

    RCLCPP_INFO(this->get_logger(),
      "Task response [%s]: success=%s, %s",
      request_id.c_str(),
      success ? "true" : "false",
      message.c_str());
  }

  // ═══════════════════════════════════════════════════════════════════
  //  Robot state aggregation → FleetState
  // ═══════════════════════════════════════════════════════════════════

  void robot_state_callback(
    const rmf_fleet_msgs::msg::RobotState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(robot_states_mutex_);

    bool is_new = (robot_states_.find(msg->name) == robot_states_.end());
    robot_states_[msg->name] = *msg;

    if (is_new) {
      RCLCPP_INFO(this->get_logger(),
        "Robot '%s' registered in fleet '%s' (pos: %.2f, %.2f)",
        msg->name.c_str(), fleet_name_.c_str(),
        msg->location.x, msg->location.y);
    }
  }

  void publish_fleet_state()
  {
    std::lock_guard<std::mutex> lock(robot_states_mutex_);

    rmf_fleet_msgs::msg::FleetState fleet_state;
    fleet_state.name = fleet_name_;

    for (const auto & [name, state] : robot_states_) {
      fleet_state.robots.push_back(state);
    }

    fleet_state_pub_->publish(fleet_state);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FleetAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
