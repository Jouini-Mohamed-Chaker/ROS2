cmake_minimum_required(VERSION 3.5)
project(line_bot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(line_follower_node src/line_follower_node.cpp)

# Link libraries
ament_target_dependencies(line_follower_node rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

# Install targets
install(TARGETS
  line_follower_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install models
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model)

ament_package()