#!/bin/bash

cd /mnt/project/build.msg/

source /mnt/reconos/tools/settings.sh
source /opt/ros/dashing/setup.bash

#build costum messages
colcon build
rm build -r -f
#rm install/my_reconros_services/lib/python3.6/ -r

rm message_package.zip -f

zip -r message_package.zip *

chmod 777 /mnt/project -R

exit
