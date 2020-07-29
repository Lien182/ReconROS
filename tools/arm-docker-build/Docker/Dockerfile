# This Dockerfile describes a simple image with rosdep installed.
# When `run`, it outputs a script for installing dependencies for a given workspace
# Requirements:
#  * mount a colcon workspace at /ws
#  * see gather_rosdeps.sh for all-caps required input environment
FROM arm32v7/ubuntu:bionic

COPY qemu-arm-static /usr/bin/qemu-arm-static

RUN apt-get update && apt-get install --no-install-recommends -y \
      dirmngr \
      gnupg2 \
    && rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list
RUN apt-get update && apt-get install --no-install-recommends -y \
      python-rosdep \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

#install ros
RUN apt-get update
RUN apt-get install --no-install-recommends -y ros-dashing-ros-base
RUN apt-get install --no-install-recommends -y nano gcc g++ libopencv-dev libeigen3-dev libpython3.6-dev libglew-dev zip

RUN rosdep init
RUN rosdep update
RUN mkdir -p /ws
WORKDIR /ws
