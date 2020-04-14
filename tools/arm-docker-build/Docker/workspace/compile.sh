#!/bin/bash

cd /mnt/project

source /mnt/reconos/tools/settings.sh

make clean
make

chmod 777 /mnt/project -R

exit