#!/usr/bin/env bash
# Kill any leftover processes from a previous explore.launch.py run.
#
# launch_ros's Shutdown does not reliably cascade SIGTERM to all
# ExecuteProcess children (gz sim, scan_filter, twist_to_stamped) or to
# nav2 lifecycle nodes spawned by navigation_launch.py. When those leak,
# multiple parameter_bridge processes accumulate and each independently
# publishes /clock from Gazebo -- the resulting interleaved /clock streams
# cause every use_sim_time node's tf2_buffer to repeatedly "jump back in
# time", which makes the costmap obstacle_layer drop scans and
# exploration stalls.
#
# Run this as a preflight in explore.launch.py.

set +e

PATTERNS=(
  parameter_bridge
  'gz sim'
  'ruby.*gz_tools'
  async_slam_toolbox
  multi_lidar_scan_merger
  aruco_map_localizer
  startup_spin
  scan_filter_node
  odom_to_tf
  twist_to_stamped
  exploration_map_saver
  '/explore_lite/lib/explore_lite/explore'
  nav2_controller/controller_server
  nav2_planner/planner_server
  nav2_behaviors/behavior_server
  nav2_bt_navigator/bt_navigator
  nav2_smoother/smoother_server
  nav2_route/route_server
  nav2_waypoint_follower/waypoint_follower
  nav2_velocity_smoother/velocity_smoother
  nav2_collision_monitor/collision_monitor
  nav2_lifecycle_manager/lifecycle_manager
  opennav_docking/opennav_docking
)

found=0
for p in "${PATTERNS[@]}"; do
  if pgrep -f "$p" > /dev/null 2>&1; then
    found=1
    break
  fi
done

if [ "$found" -eq 0 ]; then
  echo "[cleanup] no stale processes found"
  exit 0
fi

echo "[cleanup] stale processes detected from a prior run -- terminating"
for p in "${PATTERNS[@]}"; do
  pkill -TERM -f "$p" 2>/dev/null
done
sleep 2
for p in "${PATTERNS[@]}"; do
  pkill -KILL -f "$p" 2>/dev/null
done
sleep 1

remaining=0
for p in "${PATTERNS[@]}"; do
  if pgrep -f "$p" > /dev/null 2>&1; then
    echo "[cleanup] WARNING: still alive after SIGKILL: $p"
    remaining=1
  fi
done

if [ "$remaining" -eq 0 ]; then
  echo "[cleanup] done"
fi
exit 0
