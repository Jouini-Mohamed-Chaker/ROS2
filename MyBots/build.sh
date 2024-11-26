#!/bin/bash

# Check if a package name is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <package_name>"
  exit 1
fi

PACKAGE_NAME=$1

# Clean the workspace
echo "Cleaning the workspace..."
rm -rf build install log

# Rebuild the package
echo "Rebuilding the package..."
colcon build --packages-select $PACKAGE_NAME

# Source the setup file
echo "Sourcing the setup file..."
source install/setup.bash

# Run the launch file
echo "Running the launch file..."
ros2 launch $PACKAGE_NAME robot.launch.py