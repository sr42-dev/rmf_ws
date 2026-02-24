cmake_minimum_required(VERSION 3.8)
project(traffic_editor_assets)

find_package(ament_cmake REQUIRED)

# Install map files into the package's share directory
install(FILES
  map.building.yaml
  map.png
  maps.json
  DESTINATION share/${PROJECT_NAME}
)

# Install resource index marker
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(FILES
  resource/assets
  DESTINATION share/ament_index/resource_index/traffic_editor_assets
)

ament_package()
