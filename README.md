# ROS2 Jazzy Warehouse Simulation

Autonomous warehouse robot simulation using ROS2 Jazzy, Gazebo Harmonic, and the AWS RoboMaker Small Warehouse World.

Includes a Docker setup for macOS development and instructions for native Ubuntu 24.04.

---

## Option A: Native Ubuntu 24.04 (Recommended)

### 1. Install ROS2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic + ROS-GZ Bridge

```bash
sudo apt install -y ros-jazzy-ros-gz
```

### 3. Install Nav2 and Dev Tools

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git
```

### 4. Initialize rosdep

```bash
sudo rosdep init   # skip if already done
rosdep update --rosdistro=jazzy
```

### 5. Source ROS2 (add to your ~/.bashrc)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Clone and Build This Repo

```bash
# Clone the repo
git clone <YOUR_REPO_URL> ~/warehouse_ws
cd ~/warehouse_ws

# Build the workspace
colcon build
source install/setup.bash
```

### 7. Launch the Warehouse

```bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

Gazebo should open with the warehouse world (shelves, pallets, walls, clutter).

For the version without a roof (useful for top-down camera views):

```bash
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py
```

---

## Option B: Docker on macOS (Apple Silicon)

Use this if you're developing on a Mac without native Ubuntu.

### 1. Prerequisites

- Docker Desktop installed and running

### 2. Build and Run

```bash
cd <repo_directory>
docker compose build    # ~10-15 min first time
docker compose up -d
```

### 3. Open the GUI

Open **http://localhost:6080** in your browser. You'll see a desktop with a terminal.

### 4. Build and Launch (inside the container)

```bash
cd ~/ws
colcon build
source install/setup.bash
ros2 launch aws_robomaker_small_warehouse_world small_warehouse.launch.py
```

### 5. Stop

```bash
docker compose down
```

---

## What's in This Repo

```
├── src/aws-robomaker-small-warehouse-world/   # Warehouse world (adapted for Gazebo Harmonic)
│   ├── launch/          # ROS2 launch files
│   ├── models/          # 3D models (shelves, pallets, buckets, walls, etc.)
│   ├── worlds/          # Gazebo world files (.world)
│   ├── maps/            # Pre-built maps for navigation
│   └── rviz/            # RViz configs
├── Dockerfile           # Docker image (ROS2 Jazzy + Gazebo + noVNC)
├── docker-compose.yml   # Container config
├── entrypoint.sh        # Container startup script
└── config/
    └── supervisord.conf # Display stack for Docker (Xvfb + VNC + noVNC)
```

## Adaptations Made for Gazebo Harmonic

The original AWS warehouse world was built for Gazebo Classic. The following changes were made to work with Gazebo Harmonic (used by ROS2 Jazzy):

- Launch files rewritten to use `ros_gz_sim` instead of `gazebo_ros`
- Model mesh URIs changed from `file://` to `model://`
- World files updated to SDF 1.9 with Gazebo Sim system plugins
- Removed deprecated `frame=""` attributes from pose elements
- Fixed invalid inertia tensors on GroundB and RoofB models

## Next Steps

- Add a robot (TurtleBot3 or forklift URDF) and spawn it in the warehouse
- Configure Nav2 for autonomous navigation
- Add pallet detection with a camera (OpenCV / YOLO)
- Implement task logic (detect pallet, navigate, pick up, deliver)
