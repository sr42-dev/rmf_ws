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

add_executable(fleet_adapter_node src/fleet_adapter_node.cpp)

ament_target_dependencies(fleet_adapter_node
  rclcpp
  std_msgs
  rmf_fleet_msgs
  rmf_task_msgs
)

target_link_libraries(fleet_adapter_node yaml-cpp)

install(TARGETS fleet_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
