cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  worlds
  config
  rviz
  maps
  scripts
  models
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/twist_to_stamped.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install ros2-runnable scripts to lib/ so `ros2 run <pkg> <script>.py` works.
install(PROGRAMS
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
