# Agent Briefing: Autonomous Warehouse Robot Simulation

## What This Project Is

This is a university capstone project building an **autonomous warehouse robot** that can:
1. Navigate a warehouse environment autonomously
2. Detect pallets using computer vision
3. Navigate to a pallet, simulate pickup, and deliver to a drop-off point

The simulation uses **ROS2 Jazzy** on **Ubuntu 24.04** with **Gazebo Harmonic**.

---

## What Has Already Been Done (Proof of Concept on macOS Docker)

We already have a working proof of concept. The repo at `https://github.umn.edu/REISH036/RosProject.git` contains:

### Warehouse World (DONE - adapted for Gazebo Harmonic)
The AWS RoboMaker Small Warehouse World has been fully ported from Gazebo Classic to Gazebo Harmonic. Changes already made:
- Launch files rewritten to use `ros_gz_sim` instead of `gazebo_ros`
- All 14 model SDF files: mesh URIs changed from `file://models/` to `model://`
- World files updated to SDF 1.9 with Gazebo Sim system plugins (Physics, SceneBroadcaster, UserCommands, Sensors)
- Removed deprecated `frame=""` attributes from all pose elements
- Fixed invalid inertia tensors (triangle inequality violations) on GroundB and RoofB models
- `env-hooks` updated: `GAZEBO_MODEL_PATH` → `GZ_SIM_RESOURCE_PATH`
- `package.xml` dependencies: `gazebo_ros` → `ros_gz_sim` + `ros_gz_bridge`

### Docker Setup (DONE - for macOS only, not needed on Ubuntu)
The repo also contains Docker files (Dockerfile, docker-compose.yml, etc.) that were used for development on macOS. These are NOT needed on native Ubuntu — they're just there for reference.

### Verified Working
- `colcon build` succeeds
- `ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py` loads the warehouse in Gazebo Harmonic
- All models render: shelves, walls, roof, ground, pallets, pallet jack, buckets, clutter, lamps, trash cans, desks

---

## How to Set Up on Native Ubuntu 24.04

### Step 1: Install ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### Step 2: Install Gazebo Harmonic + bridge

```bash
sudo apt install -y ros-jazzy-ros-gz
```

### Step 3: Install Nav2, dev tools, and perception dependencies

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-bridge \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-opencv \
  git
```

### Step 4: Initialize rosdep

```bash
sudo rosdep init   # skip if error says already initialized
rosdep update --rosdistro=jazzy
```

### Step 5: Add ROS2 to bashrc

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Clone and build

```bash
git clone https://github.umn.edu/REISH036/RosProject.git ~/warehouse_ws
cd ~/warehouse_ws
colcon build
source install/setup.bash
```

### Step 7: Verify the warehouse world launches

```bash
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py
```

Gazebo should open showing the warehouse with shelves, pallets, walls, clutter. Use the no_roof version for better camera views.

---

## Project Architecture (What Needs to Be Built)

### Phase 1: Add a Robot to the Warehouse

**Goal**: Spawn a robot in the warehouse world and teleoperate it.

**Option A — TurtleBot3 Waffle (easiest, has LiDAR + camera)**
```bash
sudo apt install -y ros-jazzy-turtlebot3*
export TURTLEBOT3_MODEL=waffle
```
- Create a launch file that starts the warehouse world AND spawns the TurtleBot3 in it
- The TurtleBot3 Waffle comes with a 2D LiDAR and an RGB camera
- Test with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

**Option B — Custom forklift (harder, better for presentation)**
- Use the forklift URDF from https://github.com/cangozpi/ROS2-Forklift-Simulation as reference
- Add LiDAR and camera sensors to the URDF
- Create a Gazebo Harmonic spawn launch file

**Recommended**: Start with TurtleBot3 to get navigation working, optionally swap to forklift later.

### Phase 2: Navigation (Nav2)

**Goal**: Robot autonomously navigates the warehouse using Nav2 stack.

**Sub-steps**:
1. **Generate a map** using SLAM Toolbox
   - Launch warehouse + robot
   - Run `ros2 launch slam_toolbox online_async_launch.py`
   - Teleoperate robot around the warehouse to build the map
   - Save with `ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/maps/warehouse_map`
   - NOTE: The repo already has pre-built maps in `src/aws-robomaker-small-warehouse-world/maps/` that may work

2. **Configure Nav2**
   - Create a `nav2_params.yaml` config file
   - Key parameters: robot footprint, costmap sizes, planner (NavFn or SmacPlanner), controller (DWB or MPPI)
   - Create a launch file that starts: map_server, amcl (localization), planner_server, controller_server, bt_navigator

3. **Test autonomous navigation**
   - Use RViz2 to set "2D Nav Goal" and watch robot navigate
   - `ros2 launch nav2_bringup bringup_launch.py map:=/path/to/map.yaml`

### Phase 3: Pallet Detection (Perception)

**Goal**: Detect pallets in the warehouse using the robot's camera.

**Approach A — OpenCV (simpler)**
- Subscribe to the camera topic from Gazebo
- Use color/shape detection to identify pallets
- Publish detected pallet positions as ROS2 messages

**Approach B — YOLO (more impressive)**
- Use YOLOv8 with a pre-trained or fine-tuned model
- Run inference on camera images
- Publish bounding boxes and estimated positions

**Key topics to bridge from Gazebo Harmonic**:
- Camera image: use `ros_gz_bridge` to bridge the camera topic
- Example bridge config:
  ```
  ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
  ```

**Output**: A ROS2 node that publishes `geometry_msgs/PoseStamped` with pallet locations in the map frame.

### Phase 4: Task Logic (Behavior/State Machine)

**Goal**: Tie everything together — detect, navigate, pick up, deliver.

**Simple state machine**:
```
IDLE → SEARCHING → PALLET_DETECTED → NAVIGATING_TO_PALLET → AT_PALLET → PICKING_UP → NAVIGATING_TO_DROPOFF → DROPPING_OFF → IDLE
```

**Implementation options**:
- Simple Python node with states (easiest)
- `ros2` behavior tree (what Nav2 uses internally)
- SMACH or FlexBE state machine

**What "pick up" means in simulation**:
- For TurtleBot3: just navigate to the pallet position and pause (simulate attachment)
- For forklift: actuate fork joints to lift
- Either way: attach the pallet model to the robot using a Gazebo service or just track it logically

---

## Workspace Structure (Target)

```
~/warehouse_ws/
├── src/
│   ├── aws-robomaker-small-warehouse-world/   # DONE - warehouse environment
│   ├── warehouse_robot_bringup/               # TO CREATE - launch files for robot + world + nav2
│   │   ├── launch/
│   │   │   ├── warehouse_simulation.launch.py # Spawns world + robot
│   │   │   ├── navigation.launch.py           # Nav2 stack
│   │   │   └── full_system.launch.py          # Everything together
│   │   ├── config/
│   │   │   ├── nav2_params.yaml               # Nav2 configuration
│   │   │   └── bridge.yaml                    # ros_gz_bridge config
│   │   ├── maps/
│   │   │   ├── warehouse_map.pgm
│   │   │   └── warehouse_map.yaml
│   │   ├── rviz/
│   │   │   └── navigation.rviz
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── warehouse_perception/                  # TO CREATE - pallet detection
│   │   ├── warehouse_perception/
│   │   │   └── pallet_detector.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── warehouse_task_manager/                # TO CREATE - task logic / state machine
│       ├── warehouse_task_manager/
│       │   └── task_manager.py
│       ├── package.xml
│       └── setup.py
├── Dockerfile                                 # EXISTS - Docker for macOS (ignore on Ubuntu)
├── docker-compose.yml                         # EXISTS - Docker for macOS (ignore on Ubuntu)
└── README.md                                  # EXISTS
```

---

## Important Technical Details

### Gazebo Harmonic (NOT Gazebo Classic)
- This project uses **Gazebo Harmonic** (gz-sim), NOT Gazebo Classic (gazebo11)
- The command is `gz sim`, not `gazebo`
- Topics use `/world/<world_name>/...` namespace
- Sensors publish on Gazebo transport, need `ros_gz_bridge` to get them into ROS2
- Environment variable for model paths: `GZ_SIM_RESOURCE_PATH` (not `GAZEBO_MODEL_PATH`)

### ROS-Gazebo Bridge
- To get sensor data (camera, LiDAR) from Gazebo into ROS2, you MUST run `ros_gz_bridge`
- Clock topic must be bridged for `use_sim_time` to work:
  ```
  ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
  ```

### World Names
- The warehouse world is named `small_warehouse` (in the SDF)
- The no-roof variant is named `no_roof_small_warehouse`
- Gazebo topics will be under `/world/small_warehouse/...`

### Pre-existing Maps
- `src/aws-robomaker-small-warehouse-world/maps/005/map.yaml` — may work with Nav2
- Generate a fresh one with SLAM Toolbox if these don't align

---

## Priority Order

1. Get the warehouse world launching on native Ubuntu (verify the existing code works)
2. Spawn a TurtleBot3 in the warehouse
3. Teleoperate it around
4. Set up Nav2 with a map
5. Add camera bridging + pallet detection
6. Build the task manager state machine
7. (Optional) Swap TurtleBot3 for forklift model

---

## What the Professor Cares About

- **Autonomy logic** — the robot makes decisions, not a human
- **Perception** — the robot sees and identifies pallets
- **Integration** — all systems work together (nav + perception + task logic)
- **Evaluation** — show metrics (navigation success rate, detection accuracy, task completion time)
- **NOT** building URDFs or designing worlds from scratch (that's already done)
