#!/bin/bash
set -e

# Source ROS Noetic setup
source "/opt/ros/noetic/setup.bash"

# Launch the ROS talker example
roscore &

# Sleep for a few seconds to ensure the ROS core starts
sleep 5

# Run the talker example
rosrun rospy_tutorials talker

# Keep the container running
exec "$@"
