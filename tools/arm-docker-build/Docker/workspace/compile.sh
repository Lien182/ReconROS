#!/bin/bash

cd /mnt/project/build.sw/

source /mnt/reconos/tools/settings.sh
source /opt/ros/dashing/setup.bash

#build costum messages
cd msg/my_reconros_services/
colcon build
rm build -r -f
rm install/my_reconros_services/lib/python3.6/ -r

cd /mnt/project/build.sw/


#make clean
make -j$(nproc)

chmod 777 /mnt/project -R

exit
