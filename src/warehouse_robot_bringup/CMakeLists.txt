cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  config
  rviz
  maps
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/twist_to_stamped.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
