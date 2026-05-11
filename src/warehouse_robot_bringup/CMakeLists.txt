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
  scripts/odom_to_tf.py
  scripts/aruco_map_localizer.py
  scripts/startup_spin.py
  scripts/cleanup_stale_procs.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install ros2-runnable scripts to lib/ so `ros2 run <pkg> <script>.py` works.
install(PROGRAMS
  scripts/aruco_map_localizer.py
  scripts/startup_spin.py
  scripts/waypoint_follower.py
  scripts/exploration_map_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
