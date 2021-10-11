FROM arm32v7/ubuntu:bionic

COPY qemu-arm-static /usr/bin/qemu-arm-static

RUN apt-get update && apt-get install --no-install-recommends -y curl gnupg2 lsb-release dirmngr ca-certificates 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin

RUN apt update && apt install -y ca-certificates-java
#RUN dpkg --force-depends --configure ca-certificates-java

RUN apt update && apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget \
  default-jdk
# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
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
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

RUN mkdir -p /ros2_dashing/src
WORKDIR /ros2_dashing
RUN wget https://raw.githubusercontent.com/Lien182/ros2/dashing/ros2.repos
RUN vcs import src < ros2.repos


RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

RUN cd /ros2_dashing/
RUN rm -r -f src/ros-visualization src/ros2/rmw_connext src/ros2/rmw_cyclonedds src/ros2/rmw_opensplice src/eclipse-cyclonedds/cyclonedds src/ros-visualization/python_qt_binding src/ros-visualization/python_qt_binding src/ros-visualization/qt_gui_core src/ros-visualization/rqt* src/ros2/rosidl_typesupport_opensplice src/ros2/rviz
RUN colcon build --symlink-install --merge-install


RUN apt update && apt install -y nano gcc g++ libopencv-dev libeigen3-dev libpython3.6-dev libglew-dev zip

WORKDIR /ws
