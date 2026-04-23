#!/usr/bin/env bash
# build_map.sh — run Stage 2 end-to-end.
#
# Launches Gazebo + forklift + SLAM + RViz + keyboard teleop, lets you drive
# the warehouse, then saves the map via the slam_toolbox service, verifies the
# files exist, and rebuilds the workspace so the install tree picks them up.
#
# Usage:
#   ./build_map.sh                 # save as maps/warehouse.{yaml,pgm,...}
#   ./build_map.sh my_map_name     # custom name
#   NO_TELEOP=1 ./build_map.sh     # skip launching the teleop xterm (drive
#                                  # yourself from another terminal)
#
# When the map looks complete in RViz, come back to the terminal running this
# script and press ENTER to save.

set -euo pipefail

WS="${WS:-$HOME/warehouse_ws}"
MAP_NAME="${1:-warehouse}"
MAP_DIR="$WS/src/warehouse_robot_bringup/maps"
MAP_STEM="$MAP_DIR/$MAP_NAME"

if [[ ! -f "$WS/install/setup.bash" ]]; then
    echo "[build_map] ERROR: $WS/install/setup.bash not found."
    echo "[build_map] Build the workspace first:  cd $WS && colcon build --symlink-install"
    exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$WS/install/setup.bash"

mkdir -p "$MAP_DIR"

log_dir=$(mktemp -d -t build_map.XXXXXX)
echo "[build_map] Logs: $log_dir"

pids=()

cleanup() {
    echo
    echo "[build_map] Shutting down launched processes..."
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    # Give ros2 launch a chance to propagate SIGINT to its children.
    sleep 3
    for pid in "${pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    # Belt-and-suspenders for Gazebo and RViz which sometimes linger.
    pkill -f "gz sim"    2>/dev/null || true
    pkill -f "ruby .*gz" 2>/dev/null || true
    pkill -f "rviz2"     2>/dev/null || true
    pkill -f "teleop_twist_keyboard" 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[build_map] Done."
}
trap cleanup EXIT INT TERM

launch_bg() {
    local name="$1"; shift
    echo "[build_map] Starting $name..."
    ( "$@" ) > "$log_dir/$name.log" 2>&1 &
    pids+=($!)
}

# 1) Gazebo + forklift + controllers + bridges.
launch_bg sim ros2 launch warehouse_robot_bringup warehouse_simulation.launch.py

# Wait for controllers to come up before starting SLAM (they spawn at ~8s, plus
# bridge init). If SLAM starts before /scan exists it'll just log warnings, but
# this keeps the logs clean.
echo "[build_map] Waiting 15 s for Gazebo + controllers to initialize..."
sleep 15

# 2) SLAM (online async).
launch_bg slam ros2 launch warehouse_robot_bringup slam.launch.py

# 3) RViz.
launch_bg rviz ros2 launch warehouse_robot_bringup view_robot.launch.py

# 4) Teleop (unless suppressed).
if [[ -z "${NO_TELEOP:-}" ]]; then
    launch_bg teleop ros2 launch warehouse_robot_bringup teleop.launch.py
else
    echo "[build_map] NO_TELEOP set — drive the robot from your own terminal."
fi

cat <<EOF

================================================================================
  STAGE 2 — drive the warehouse
================================================================================
  * RViz:  change Fixed Frame from "odom" to "map" to watch the map grow.
  * Teleop: focus the xterm window; use i/j/k/l/u/o/m/,/. to drive.
            (q/w/e are speed multipliers — they don't move the robot.)
  * Drive SLOWLY down every aisle so scans overlap and loop closures fire.

  When the map looks complete, come back to THIS terminal and press ENTER
  to save as: $MAP_STEM.{yaml,pgm,posegraph,data}

  Ctrl-C here will abort without saving.
================================================================================
EOF

read -r _ || true

# Verify /map is actually publishing before attempting the save (the most
# common silent-failure cause of Stage 2 is calling save before SLAM has
# anything to give).
echo "[build_map] Verifying /map is publishing..."
if ! timeout 5 ros2 topic echo --once /map >/dev/null 2>&1; then
    echo "[build_map] ERROR: /map is not publishing. Is SLAM running? Check $log_dir/slam.log"
    exit 1
fi
echo "[build_map] /map OK."

# Save via the slam_toolbox service (more reliable than nav2 map_saver_cli,
# which has a 2 s timeout that often trips against transient-local /map).
echo "[build_map] Saving occupancy grid to $MAP_STEM.{yaml,pgm}..."
save_out=$(ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap \
    "{name: {data: '$MAP_STEM'}}")
echo "$save_out"
if ! grep -q "result=0" <<< "$save_out"; then
    echo "[build_map] ERROR: save_map did not return result=0."
    exit 1
fi

# Serialize the posegraph too (useful for resuming mapping later).
echo "[build_map] Serializing posegraph to $MAP_STEM.{posegraph,data}..."
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$MAP_STEM'}" || true

# Confirm the files exist on disk.
if [[ ! -f "$MAP_STEM.yaml" || ! -f "$MAP_STEM.pgm" ]]; then
    echo "[build_map] ERROR: save reported success but files are missing:"
    ls -l "$MAP_DIR" || true
    exit 1
fi

echo "[build_map] Map files:"
ls -l "$MAP_STEM".{yaml,pgm,posegraph,data} 2>/dev/null || ls -l "$MAP_STEM".{yaml,pgm}

# Rebuild so install/ picks up the new map.
echo "[build_map] Rebuilding warehouse_robot_bringup so install/ sees the map..."
( cd "$WS" && colcon build --symlink-install --packages-select warehouse_robot_bringup )

echo
echo "[build_map] ============================================================"
echo "[build_map] Map saved. Stage 3 is ready to run:"
echo "[build_map]"
echo "[build_map]   source ~/warehouse_ws/install/setup.bash"
echo "[build_map]   ros2 launch warehouse_robot_bringup navigation.launch.py"
echo "[build_map] ============================================================"
