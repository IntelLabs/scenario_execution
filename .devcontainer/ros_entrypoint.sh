#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# ros2_workspace
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
else
    echo "/workspace/install/setup.bash not found"
fi

exec "$@"
