#!/bin/bash

# $1 target architecture (arm32, arm64)
# $2 target ROS 2 distribution (dashing, foxy)

if [ "$1" == "" || "$2" == "" ]
then
  echo "Usage: $0  <arm32,arm64> <dashing,foxy>"
  exit
fi

docker run -it --rm  --name reconros_$1_$2_inst_2_0 -v /usr/bin/qemu-arm-static:/usr/bin/qemu-arm-static -v $(pwd)/:/mnt/project:rw -v $RECONOS:/mnt/reconos:ro reconros_$1_$2:2.0 bash /mnt/reconos/tools/arm-docker-build/Docker/workspace/compile_msg.sh
