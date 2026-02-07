#!/bin/bash
set -euo pipefail

# Source ROS environment
if [ "$ROS_TYPE" = "ros1" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    source $PWD/catkin_ws/devel/setup.bash
else
    source /opt/ros/$ROS_DISTRO/setup.bash
    source $PWD/install/setup.bash
fi

# Export mock mode
if [ "$MOCK_MODE" = "true" ]; then
    export MOCK_MODE=true
    echo "Mock mode enabled"
fi

exec "$@"