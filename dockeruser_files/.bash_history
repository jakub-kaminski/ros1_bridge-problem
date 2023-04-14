ros2 run examples_rclcpp_minimal_publisher publisher_lambda
source install/setup.bash
source /ros1_bridge/install/setup.bash
source /ros2_humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /opt/ros/humble/setup.bash
