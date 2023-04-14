#!/bin/bash
set -e

# Source ROS Noetic setup
source "/opt/ros/noetic/setup.bash"


# Keep the container running
exec "$@"
