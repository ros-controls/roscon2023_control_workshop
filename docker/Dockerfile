ARG ROS_DISTRO="rolling"
FROM ros:${ROS_DISTRO}

RUN apt-get update && apt-get install -y git tig vim build-essential && \
    rm -rf /var/lib/apt/lists/*

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-rviz2 && \
    rm -rf /var/lib/apt/lists/*

# tiago-related deps
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-launch-param-builder ros-${ROS_DISTRO}-joint-state-publisher-gui && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_LOCALHOST_ONLY=TRUE

RUN mkdir -p /home/ros/ws/src
