# ros1_bridge-problem

## Description
This repository is an attempt to build Ubuntu `22.04 Jammy` with ROS2 `humble`, Webots simulator, and `ros1_bridge` in a Docker devel container feasible for building with `colcon`.


### **Problem statement:**

**How to make a docker development image that will contain Webots dependencies, allow for build the workspace packages by a developer, and be able to run ros1_bridge?**

### Important notes:
Official guide: [Using ros1_bridge with upstream ROS on Ubuntu]( https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html )

**Current image build consumes 16GB and takes ~1.5h to build (ROS2 from sources).**

Please note that `ros1_bridge` and `rosbridge` are totally different, unrelated packages.

Please consider building docker image targets `dev` and `dev-2`.
Both are for ROS2 devel image, and currently fail.


### TODO:
- Solve the problem: after installing `ros1_bridge`, `colcon build` fails, there are unmet dependencies for Webots simulator (but Webots was installed before). Example ros2 workspace is provided for tests.
- Show how to correctly use ROS2 binaries in a multistage build such that they are combined with ros1_bridge that cannot be otherwise built with ros2_binaries on a non-docker system.
- Reduce image size and build time


### Use

```
./build-ros2-image.sh
./run-ros2-image.sh
./enter-ros2-container.sh
```
**TIP:** Use arrow keys to navigate through a shell commnds history preloaded to the container.*

### Common Error messages:

Trying to build the workspace:
Example missing packges:
- `ament_lint` 
- `webots_ros2_driver`
- and others

After:
```
source /opt/ros/humble/local_setup.bash
```
Workspace builds, but now:

Getting the following issues when trying to start ros1_bridge:

```
source /opt/ros/humble/setup.bash
dockeruser@dell:~/ros2_ws$ ros2 run ros1_bridge dynamic_bridge
/ros1_bridge/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge: error while loading shared libraries: libroscpp.so.4d: cannot open shared object file: No such file or directory
[ros2run]: Process exited with failure 127
```

If you are able to get `"ERROR UNABLE TO CONTACT LOCAL HOST 13111..."` at `ros1_bridge` startup, it means that ros1_bridge will work well if set up correctly. If you are able to get this "unable" error at the same time when workspace is built correctly, you have solved the problems.

