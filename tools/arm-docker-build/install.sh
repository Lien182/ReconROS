#!/bin/bash

# $1 target architecture (arm32, arm64)
# $2 target ROS 2 distribution (dashing, foxy)

if [ "$1" == "" || "$2" == "" ]
then
  echo "Usage: $0  <arm32,arm64> <dashing,foxy>"
  exit
fi

cp Docker/qemu-arm-static Docker/$1_$2/
docker image build -t reconros_$1_$2:2.0 Docker/$1_$2/
docker run -it --rm  --name reconros_$1_$2_inst_2_0 -v /usr/bin/qemu-arm-static:/usr/bin/qemu-arm-static -v $(pwd)/Docker/workspace:/mnt/workspace:rw reconros_$1_$2:2.0 bash /mnt/workspace/workspace.sh