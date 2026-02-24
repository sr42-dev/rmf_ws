# Robots CV Simulator

A ROS 2 simulator using OpenCV for visualization.

## Usage

1. Build the package:
   ```bash
   colcon build --packages-select robots_cv_sim
   source install/setup.bash
   ```

2. Run the simulator:
   ```bash
   ros2 launch robots_cv_sim sim.launch.py
   ```

   Arguments:
   - `num_robots`: Number of robots (default 100)
   - `map_yaml`: Path to map yaml
   - `map_img_path`: Path to map image

3. Run the random walk test:
   ```bash
   python3 src/robots_cv_sim/test/random_walk.py
   ```
