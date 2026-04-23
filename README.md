# ROS 2 Jazzy Warehouse Forklift

Autonomous forklift in the AWS RoboMaker Small Warehouse, built in four incremental stages. Each stage has explicit pass criteria — do not move to the next stage until the current one works.

**Stack:** ROS 2 Jazzy · Gazebo Harmonic · `ros2_control` · `slam_toolbox` · Nav2 · `nav2_simple_commander`

**Workspace packages** (`src/`):
- `warehouse_robot_bringup/` — forklift URDF (diff drive + LiDAR + camera + prismatic fork), controllers, launch files, and per-stage configs.
- `aws-robomaker-small-warehouse-world/` — warehouse SDF world (adapted for Gazebo Harmonic).

---

## Setup (native Ubuntu 24.04)

### 1. Install ROS 2 Jazzy + Gazebo Harmonic + Nav2

```bash
# Locale + ROS 2 apt source
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-xacro \
  python3-colcon-common-extensions \
  python3-rosdep \
  xterm \
  git

sudo rosdep init  # skip if already done
rosdep update --rosdistro=jazzy

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone and build

```bash
git clone https://github.umn.edu/REISH036/RosProject.git ~/warehouse_ws
cd ~/warehouse_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/warehouse_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Every new terminal needs `source ~/warehouse_ws/install/setup.bash` (already in `~/.bashrc` after the step above).

---

## Stage 1 — Drive the forklift in simulation

**Goal:** `cmd_vel` moves the robot; `/diff_cont/odom`, TF, and `/scan` all publish correctly.

Three terminals:

```bash
# T1 — Gazebo + forklift + controllers + sensor bridge
ros2 launch warehouse_robot_bringup warehouse_simulation.launch.py

# T2 — RViz (RobotModel / TF / LaserScan / Odometry / Map pre-configured)
ros2 launch warehouse_robot_bringup view_robot.launch.py

# T3 — Keyboard teleop (opens in xterm)
ros2 launch warehouse_robot_bringup teleop.launch.py
```

In the teleop xterm, use `i / , / j / l / u / o / m / .` to drive (not `q / w / e` — those are only speed multipliers).

**Pass criteria** (in a 4th terminal):

```bash
ros2 topic hz /scan                      # ~10 Hz
ros2 topic echo /diff_cont/odom --once   # pose updates while driving
ros2 run tf2_tools view_frames           # odom → base_link → base_footprint → wheels/laser_frame_link/camera_link
```

Robot model must translate in RViz as you drive in Gazebo.

### How the drive plumbing works

Jazzy's `diff_drive_controller` only accepts `geometry_msgs/TwistStamped`, but teleop and Nav2 publish plain `Twist`. A small relay (`scripts/twist_to_stamped.py`, started by `warehouse_simulation.launch.py`) converts `/cmd_vel` → `/diff_cont/cmd_vel`. Publish on `/cmd_vel` — nothing else.

---

## Stage 2 — Build a map with SLAM

**Goal:** Drive the warehouse and produce a saved occupancy grid.

Four terminals (keep T1/T2 from Stage 1 running):

```bash
# T1 — sim (already running from Stage 1)
ros2 launch warehouse_robot_bringup warehouse_simulation.launch.py

# T2 — RViz (already running; change Fixed Frame from "odom" to "map")
ros2 launch warehouse_robot_bringup view_robot.launch.py

# T3 — slam_toolbox (online async)
ros2 launch warehouse_robot_bringup slam.launch.py

# T4 — teleop: drive SLOWLY around every aisle so loop closures happen
ros2 launch warehouse_robot_bringup teleop.launch.py
```

**Save the map** in a 5th terminal when coverage looks good:

```bash
cd ~/warehouse_ws
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'src/warehouse_robot_bringup/maps/warehouse'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: 'src/warehouse_robot_bringup/maps/warehouse'}"
colcon build --symlink-install --packages-select warehouse_robot_bringup
```

(Prefer the `slam_toolbox` save service over `nav2_map_server map_saver_cli` — the CLI has a short timeout that times out against slam_toolbox's transient-local `/map` publisher.)

**Pass criteria:**
- `/map` grows live in RViz, no visible double walls.
- `src/warehouse_robot_bringup/maps/warehouse.{yaml,pgm,posegraph,data}` all exist.

**Rule:** do **not** run Nav2 while mapping. Map first → save → then navigate on the saved map.

---

## Stage 3 — Localize and navigate (planned)

**Goal:** Launch stack, set initial pose in RViz, click a Nav2 goal, forklift plans and reaches it.

Planned artifacts (not yet in the repo):
- `config/nav2_params.yaml` — AMCL + costmaps + controller tuned to the forklift.
- `launch/navigation.launch.py` — includes the Gazebo sim + `nav2_bringup/bringup_launch.py` with the saved map.

Run (once implemented):
```bash
ros2 launch warehouse_robot_bringup navigation.launch.py
# In RViz: "2D Pose Estimate" near the real pose, then "Nav2 Goal" to click goals.
```

**Pass:** 3–5 goals across the warehouse reached autonomously; AMCL particle cloud converges; no costmap frame errors.

---

## Stage 4 — Autonomous waypoint tour (planned)

**Goal:** A single ROS 2 node drives the forklift through named waypoints (`loading_zone`, `aisle_1`, `aisle_2`, `drop_zone`) with no human clicks.

Planned artifacts:
- `config/waypoints.yaml` — named poses in the `map` frame.
- `scripts/waypoint_follower.py` — uses `nav2_simple_commander.BasicNavigator.followWaypoints()`.
- `launch/waypoint_tour.launch.py` — Stage-3 navigation + the waypoint node, with `route` and `loop` params.

Run (once implemented):
```bash
ros2 launch warehouse_robot_bringup waypoint_tour.launch.py
```

---

## Repo layout

```
warehouse_ws/
├── src/
│   ├── warehouse_robot_bringup/
│   │   ├── urdf/          # forklift.urdf.xacro + sensors + ros2_control
│   │   ├── config/        # forklift_controllers.yaml, slam_toolbox_async.yaml
│   │   ├── launch/        # warehouse_simulation, view_robot, teleop, slam
│   │   ├── rviz/          # forklift.rviz (RobotModel + TF + Scan + Odom + Map)
│   │   ├── scripts/       # twist_to_stamped.py (Twist → TwistStamped relay)
│   │   └── maps/          # slam_toolbox output lands here
│   └── aws-robomaker-small-warehouse-world/
│       ├── worlds/        # small_warehouse.world, no_roof_small_warehouse.world (SDF 1.9)
│       ├── models/        # shelves, pallets, walls, clutter
│       └── launch/        # world-only launch files
├── Dockerfile             # optional Docker setup for macOS dev
└── docker-compose.yml
```

## Gazebo Harmonic adaptations

The original AWS warehouse was built for Gazebo Classic. Changes to run on Harmonic / ROS 2 Jazzy:
- Launch files use `ros_gz_sim` instead of `gazebo_ros`.
- Model mesh URIs: `file://` → `model://`.
- Worlds upgraded to SDF 1.9 with Gazebo Sim system plugins.
- Removed deprecated `frame=""` attributes.
- Fixed invalid inertia tensors on `GroundB` / `RoofB`.

## Docker on macOS (Apple Silicon)

```bash
cd <repo>
docker compose build   # ~10–15 min first time
docker compose up -d
```

Open <http://localhost:6080> for a noVNC desktop, then inside:

```bash
cd ~/ws && colcon build && source install/setup.bash
ros2 launch warehouse_robot_bringup warehouse_simulation.launch.py
```

Stop with `docker compose down`.
