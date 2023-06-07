FROM arm64v8/ubuntu:focal

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

# Requirement: qemu-user-static installed on host
COPY ./qemu-aarch64-static /usr/bin/qemu-aarch64-static

# Update ubuntu
RUN apt-get update && apt-get upgrade -y

# Install ROS
## Add ROS to apt
RUN apt-get update && \
    apt-get install --no-install-recommends -y curl gnupg lsb-release ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install ROS base
RUN apt-get update && \
    apt-get install --no-install-recommends -y ros-galactic-ros-base ros-galactic-rmw-fastrtps-cpp

# Install build requirements for ReconROS
RUN apt-get update && \
    apt-get install --no-install-recommends -y build-essential zip python3-colcon-common-extensions python3-setuptools libopencv-dev

RUN mkdir /ros2_build/ && ln -s /opt/ros/galactic/ /ros2_build/install

WORKDIR /ws
