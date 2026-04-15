#!/bin/bash
set -e

# Start the display stack (Xvfb + fluxbox + x11vnc + noVNC)
supervisord -c /etc/supervisor/conf.d/supervisord.conf &

# Wait for display to be ready
sleep 2

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace overlay if built
if [ -f "$HOME/ws/install/setup.bash" ]; then
    source "$HOME/ws/install/setup.bash"
fi

# Execute the command passed to docker run (default: bash)
exec "$@"
