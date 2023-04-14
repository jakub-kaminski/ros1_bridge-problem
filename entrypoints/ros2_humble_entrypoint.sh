#!/bin/bash
set -e

# setup ros2 environment
source "/ros1_bridge/install/setup.bash"
source "/opt/ros/humble/setup.bash"
export ROS_MASTER_URI=http://ros1_container:11311

exec "$@"
