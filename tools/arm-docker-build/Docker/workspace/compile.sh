#!/bin/bash

cd /mnt/project/build.sw/

source /mnt/reconos/tools/settings.sh

#make clean
make -j$(nproc)

chmod 777 /mnt/project -R

exit