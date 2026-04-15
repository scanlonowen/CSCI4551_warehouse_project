FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── Layer 1: ROS2 Desktop ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# ── Layer 2: Gazebo Harmonic + ROS-GZ bridge ──────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    mesa-utils \
    libgl1-mesa-dri \
    libegl-mesa0 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

# ── Layer 3: noVNC display stack ──────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    fluxbox \
    supervisor \
    novnc \
    websockify \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# ── Layer 4: Dev tools + Nav2 ─────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    vim \
    wget \
    curl \
    htop \
    tmux \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update --rosdistro=jazzy || true

# ── Layer 5: User setup ──────────────────────────────────────────────
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y sudo \
    && existing_user=$(getent passwd $USER_UID | cut -d: -f1) \
    && if [ -n "$existing_user" ] && [ "$existing_user" != "$USERNAME" ]; then \
         usermod -l $USERNAME -d /home/$USERNAME -m $existing_user; \
         groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) 2>/dev/null || true; \
       elif [ -z "$existing_user" ]; then \
         groupadd --gid $USER_GID $USERNAME 2>/dev/null || true; \
         useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
       fi \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && chsh -s /bin/bash $USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Copy supervisor config
COPY config/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create workspace
RUN mkdir -p /home/$USERNAME/ws/src && chown -R $USERNAME:$USERNAME /home/$USERNAME/ws

# Setup bashrc for ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash' >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME/ws

ENV DISPLAY=:1
ENV ROS_DOMAIN_ID=0
ENV GZ_VERSION=harmonic

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
