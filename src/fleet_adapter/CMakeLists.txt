cmake_minimum_required(VERSION 3.8)
project(fleet_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fleet_adapter_node src/fleet_adapter_node.cpp)

ament_target_dependencies(fleet_adapter_node
  rclcpp
  std_msgs
  rmf_fleet_msgs
  rmf_task_msgs
  rmf_traffic_ros2
)

target_link_libraries(fleet_adapter_node
  yaml-cpp
  rmf_traffic::rmf_traffic
  Eigen3::Eigen
)

install(TARGETS fleet_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
