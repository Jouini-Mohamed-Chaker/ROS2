cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

# Include directories
include_directories(${CMAKE_BINARY_DIR})

# Add executable
if (EXISTS "${CMAKE_SOURCE_DIR}/src/lidar_node.cpp")
  add_executable(lidar_node src/lidar_node.cpp)
  target_link_libraries(lidar_node ignition-transport${IGN_TRANSPORT_VER}::core)
endif()

# Link libraries
ament_target_dependencies(lidar_node rclcpp sensor_msgs geometry_msgs)

# Install targets
install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install models
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model)

ament_package()