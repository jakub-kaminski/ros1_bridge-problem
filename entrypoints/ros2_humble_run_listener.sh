#!/bin/bash
set -e

# setup ros2 environment
source "/ros1_bridge/install/setup.bash"
source "/opt/ros/humble/setup.bash"
export ROS_MASTER_URI=http://ros1_container:11311

sleep 7

ros2 run ros1_bridge dynamic_bridge &

sleep 3

ros2 run demo_nodes_cpp listener

exec "$@"
