# This Dockerfile describes a simple image with rosdep installed.
# When `run`, it outputs a script for installing dependencies for a given workspace
# Requirements:
#  * mount a colcon workspace at /ws
#  * see gather_rosdeps.sh for all-caps required input environment

#foxy only for ubuntu 20
FROM arm32v7/ubuntu:focal

COPY qemu-arm-static /usr/bin/qemu-arm-static

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

RUN apt-get update && apt-get install --no-install-recommends -y curl gnupg2 lsb-release dirmngr ca-certificates 

RUN update-ca-certificates -f

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

RUN apt update && apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install -y ca-certificates-java
#RUN dpkg --force-depends --configure ca-certificates-java

RUN apt update && apt install -y \
  build-essential \
  git \
  libssl-dev \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-vcstool \
  python3-testresources \
  python3-catkin-pkg-modules\
  wget


#install cmake

RUN wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2.tar.gz && tar -zxvf cmake-3.23.2.tar.gz && cd cmake-3.23.2 && ./bootstrap && make && make install 


# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools==58.2.0

RUN python3 -m pip install -U importlib-metadata importlib-resources

# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libcunit1-dev

RUN mkdir -p /ros2_build/src
WORKDIR /ros2_build
RUN wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos --no-check-certificate
RUN vcs import src < ros2.repos


RUN c_rehash /etc/ssl/certs

RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

RUN rm src/eclipse-cyclonedds -r -f
RUN rm src/eclipse-iceoryx/ -r -f
RUN rm src/ros2/rmw_connextdds -r -f
RUN rm src/ros2/rmw_cyclonedds/ -r -f
RUN rm src/ros2/demos -r -f
RUN rm src/ros2/examples -r -f


RUN cd /ros2_build/
RUN chmod 777 -R .
#RUN rm -r -f src/ros-visualization src/ros2/rmw_connext src/ros2/rmw_cyclonedds src/ros2/rmw_opensplice src/eclipse-cyclonedds/cyclonedds src/ros-visualization/python_qt_binding src/ros-visualization/python_qt_binding src/ros-visualization/qt_gui_core src/ros-visualization/rqt* src/ros2/rosidl_typesupport_opensplice src/ros2/rviz
RUN colcon build --symlink-install --merge-install


RUN apt update && apt install -y nano gcc g++ libopencv-dev libeigen3-dev libglew-dev zip

WORKDIR /ws

ENV DEBIAN_FRONTEND teletype

#----------------------------------------------------------



