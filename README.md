# RMF Robot Navigation Workspace

A ROS 2 Jazzy workspace for simulating and testing autonomous robot navigation using the [Open-RMF](https://www.open-rmf.org/) framework. Robots navigate a 2D map along pre-defined lanes, controlled via `PathRequest` messages, and validated by an automated integration test.

## Purpose

This workspace is designed as a **sandbox for experimenting with the RMF stack**. It provides a lightweight, self-contained environment where you can:

- **Prototype fleet adapters** — write and iterate on `rmf_fleet_adapter` Python or C++ implementations against a working simulator, without needing real hardware. Boilerplate code and examples can be found in the [fleet_adapter_template](https://github.com/open-rmf/fleet_adapter_template) and [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) repositories.
- **Develop robot client APIs** — build the `RobotClientAPI` shim that translates RMF commands (navigate, stop, dock) into your robot's native protocol and test it end-to-end.
- **Integrate with `rmf_core`** — the container ships with `rmf_traffic_ros2`, `rmf_task_ros2`, `rmf_fleet_msgs`, and the traffic editor, so you can experiment with traffic deconfliction, task dispatch, and schedule negotiation out of the box.
- **Validate navigation logic** — use the included OpenCV simulator and automated test to verify path-following, collision avoidance, and multi-robot coordination before deploying to real robots.
- **Edit maps interactively** — modify `map.building.yaml` (vertices, lanes, walls) and immediately re-run the test to see the effect on robot routing.

The goal is to give you a fast feedback loop: change code → rebuild → launch → observe results — all inside a reproducible Docker container.

## Prerequisites

- **Docker** (with Compose v2)
- **NVIDIA GPU + drivers** (for GPU-accelerated containers)
- **X11** display server (for the simulator GUI)

## Workspace Structure

```
rmf_ws/
├── .devcontainer/          # Dev container configuration
│   ├── devcontainer.json   # VS Code devcontainer settings
│   ├── docker-compose.yml  # Compose service definition
│   └── Dockerfile          # ROS 2 Jazzy image + RMF deps
├── src/
│   ├── traffic_editor_assets/  # Map definition (building YAML, lanes, vertices)
│   ├── robots_cv_sim/          # 2D robot simulator (OpenCV-based)
│   ├── robot_controller/       # C++ controller node (SPIN → DRIVE waypoint following)
│   ├── rmf_bringup/            # Launch file + config generation scripts
│   └── rmf_tests/              # Integration tests (navigation test)
└── README.md
```

### Package Descriptions

| Package | Language | Description |
|---------|----------|-------------|
| `traffic_editor_assets` | CMake | Map data — `map.building.yaml` with vertices, lanes, and wall geometry |
| `robots_cv_sim` | Python | OpenCV-based 2D simulator that publishes robot pose, laser scan, and accepts velocity commands |
| `robot_controller` | C++ | Waypoint-following controller that subscribes to `PathRequest` and drives robots through waypoints |
| `rmf_bringup` | Python | Main launch file that generates configs, starts the simulator, controllers, and test |
| `rmf_tests` | Python | Navigation integration test — sends a `PathRequest` with 15 waypoints and validates arrival + collision-free travel |

## Quick Start

### Option A: VS Code Dev Container (recommended)

1. **Open the workspace** in VS Code:
   ```bash
   code /path/to/rmf_ws
   ```

2. **Reopen in Container** — when prompted, click "Reopen in Container", or run the command:
   > Dev Containers: Reopen in Container

3. **Build the workspace** inside the container terminal:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd /home/dev/rmf_ws
   colcon build --symlink-install
   ```

4. **Allow X11 forwarding** (on the host, before launching):
   ```bash
   xhost +local:docker
   ```

5. **Run the navigation test**:
   ```bash
   source install/setup.bash
   ros2 launch rmf_bringup bringup.launch.py test_name:=test_navigation
   ```

### Option B: Docker Compose (command line)

1. **Build and start the container**:
   ```bash
   cd .devcontainer
   docker compose up --build -d
   ```

2. **Allow X11 forwarding** (on the host):
   ```bash
   xhost +local:docker
   ```

3. **Build the ROS 2 workspace** inside the container:
   ```bash
   docker exec devcontainer-rmf-jazzy-dev-1 bash -c \
     "source /opt/ros/jazzy/setup.bash && \
      cd /home/dev/rmf_ws && \
      colcon build --symlink-install"
   ```

4. **Run the navigation test**:
   ```bash
   docker exec devcontainer-rmf-jazzy-dev-1 bash -c \
     "source /opt/ros/jazzy/setup.bash && \
      source /home/dev/rmf_ws/install/setup.bash && \
      ros2 launch rmf_bringup bringup.launch.py test_name:=test_navigation"
   ```

5. **Stop the container** when done:
   ```bash
   docker compose down
   ```

## Launch Arguments

The bringup launch file accepts the following arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `num_robots` | `1` | Number of robots to spawn |
| `test_name` | `test_navigation` | Test executable from `rmf_tests` |
| `test_delay` | `5.0` | Seconds to wait before starting the test |
| `map_yaml` | *(bundled map)* | Path to `map.building.yaml` |
| `map_img_path` | *(bundled image)* | Path to the map PNG image |
| `start_config` | *(bundled config)* | Path to robot spawn configuration |

## What the Navigation Test Does

The `test_navigation` node:

1. Waits for the simulator to publish robot pose data
2. Selects 15 waypoints from the map's vertex data
3. Sends a single `PathRequest` message with all waypoints
4. Monitors robot progress — verifying each waypoint is reached (within tolerance)
5. Checks for collisions via laser scan data
6. Reports results:

```
============================================================
TEST SUMMARY
============================================================
Waypoints reached: 15/15
Total collisions: 0

✓ ALL TESTS PASSED
```

## Launch Sequence

The bringup launch file orchestrates the following steps:

```
t=0s   Generate nav graph + fleet config from map.building.yaml
t=0s   Start robot simulator (robots_cv_sim)
t=1s   Start robot controller(s) (robot_controller)
t=5s   Start navigation test (rmf_tests)
```
