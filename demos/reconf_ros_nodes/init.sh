source /opt/ros/dashing/setup.sh
OLD_PATH=$(pwd)
cd /opt/reconos/
cat download.bit > /dev/xdevcfg
./reconos_init.sh
cd $OLD_PATH
