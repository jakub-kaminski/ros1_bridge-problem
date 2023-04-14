FROM ubuntu:jammy AS secret

RUN DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        curl gnupg lsb-release &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential \
      cmake \
      git \
      python3-flake8 \
      python3-flake8-blind-except \
      python3-flake8-builtins \
      python3-flake8-class-newline \
      python3-flake8-comprehensions \
      python3-flake8-deprecated \
      python3-flake8-docstrings \
      python3-flake8-import-order \
      python3-flake8-quotes \
      python3-pip \
      python3-pytest \
      python3-pytest-cov \
      python3-pytest-repeat \
      python3-pytest-rerunfailures \
      python3-rosdep2 \
      python3-setuptools \
      locales \
      wget

RUN python3 -m pip install -U colcon-common-extensions vcstool

RUN locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble
RUN wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
RUN vcs import src < ros2.repos
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --rosdistro humble \
      --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
RUN colcon build --symlink-install


# Begin WEBOTS INSTALLATION
ARG UID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -d /dockeruser -m \
            -u $UID -U \
            -s /usr/bin/bash \
            -c "Docker User" dockeruser && \
    echo 'dockeruser ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    usermod -a -G video dockeruser && \
    usermod -a -G dialout dockeruser

# Webots_Docker
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    ros-humble-rqt-common-plugins \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-moveit-ros \
    ros-humble-joint-state-publisher \
    ros-humble-webots-ros2 \
    wget \
    vim \
    gdb

RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2023a/webots_2023a_amd64.deb -O /tmp/webots.deb && \
    apt-get install -y /tmp/webots.deb && \
    rm /tmp/webots.deb

COPY --chown=dockeruser dockeruser_files/Webots-R2023a.conf /dockeruser/.config/Cyberbotics/Webots-R2023a.conf

# END Webots_Installation

# INSTALL ROS1_BRIDGE
# Please see: https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
# Some limitations in building and ROS1 & ROS2 package conflicts apply (when using binaries).
FROM secret AS ros1_bridge-builder

RUN rm /etc/apt/sources.list.d/ros2.list &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt remove -y \
        python3-catkin-pkg python3-catkin-pkg-modules &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-core-dev

RUN mkdir -p /ros1_bridge/src
WORKDIR /ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge
RUN /bin/bash -c ". /ros2_humble/install/local_setup.bash &&\
                  colcon build"


# Unable to compile colcon packages when runing a container of this image
FROM secret AS dev
COPY --from=ros1_bridge-builder /ros1_bridge  /ros1_bridge

WORKDIR /
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=humble
COPY entrypoints/ros2_humble_entrypoint.sh /
ENTRYPOINT ["/ros2_humble_entrypoint.sh"]
CMD ["bash"]

USER dockeruser
WORKDIR /dockeruser/ros2_ws

COPY ros2_ws/src /dockeruser/ros2_ws/src
COPY --chown=dockeruser dockeruser_files/.bash_history /dockeruser/.bash_history
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY dockeruser_files/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /dockeruser/.bashrc

# Alternative.
# Unable to compile colcon packages when runing a container of this image

FROM secret AS dev-2

RUN rm /etc/apt/sources.list.d/ros2.list &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt remove -y \
        python3-catkin-pkg python3-catkin-pkg-modules &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-core-dev

RUN mkdir -p /ros1_bridge/src
WORKDIR /ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge
RUN /bin/bash -c ". /ros2_humble/install/local_setup.bash &&\
                  colcon build"

WORKDIR /
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=humble
COPY entrypoints/ros2_humble_entrypoint.sh /
ENTRYPOINT ["/ros2_humble_entrypoint.sh"]
CMD ["bash"]

USER dockeruser
WORKDIR /dockeruser/ros2_ws

COPY ros2_ws/src /dockeruser/ros2_ws/src
COPY --chown=dockeruser dockeruser_files/.bash_history /dockeruser/.bash_history
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY dockeruser_files/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /dockeruser/.bashrc


#FROM secret AS pre-deploy-2

#RUN rm /etc/apt/sources.list.d/ros2.list &&\
    #DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    #DEBIAN_FRONTEND=noninteractive apt remove -y \
        #python3-catkin-pkg python3-catkin-pkg-modules &&\
    #DEBIAN_FRONTEND=noninteractive apt-get install -y \
        #ros-core-dev

#RUN mkdir -p /ros1_bridge/src
#WORKDIR /ros1_bridge
#RUN git clone https://github.com/ros2/ros1_bridge
#RUN /bin/bash -c ". /opt/ros/humble/setup.bash &&\
                  #colcon build"


#FROM secret AS deploy-2

#COPY --from=pre-deploy-2 /ros1_bridge /ros1_bridge

#WORKDIR /
#ENV ROS1_DISTRO=noetic
#ENV ROS2_DISTRO=humble
#COPY ros_entrypoint.sh /
#ENTRYPOINT ["/ros2_humble_entrypoint.sh"]
#CMD ["bash"]

#USER dockeruser
#WORKDIR /dockeruser/ros2_ws
