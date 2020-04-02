#!/bin/bash

sudo apt install qemu-user-static

cp /usr/bin/qemu-arm-static Docker/ 
sudo docker image build -t ros_arm:1.0 Docker/
sudo docker run -it --rm  --name ros_arm_inst -v /usr/bin/qemu-arm-static:/usr/bin/qemu-arm-static -v $(pwd)/Docker/workspace:/mnt/workspace:rw ros_arm:1.0 bash /mnt/workspace/workspace.sh