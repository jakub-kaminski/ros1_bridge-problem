docker container rm -f ros1_bridge-problem-container 2> /dev/null 1> /dev/null

docker run \
    --net=host \
    --cap-add SYS_ADMIN \
    --name ros1_bridge-problem-container \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -d --rm -it ros1_bridge-problem-image


    #-v ./ros2_ws:/dockeruser/ros2_ws \
