#/bin/bash

[[ -z "$1" ]] && { echo "Parameter 1 is empty" ; return 1; }

rm $1 -f
rm $1.* -f
rm download.bit -f
rm download.bit.* -f

wget 192.168.2.2:8082/download.bit
wget 192.168.2.2:8082/build.sw/$1

cat download.bit > /dev/xdevcfg

chmod +1 $1 


[[ -z "$2" ]] && { echo "No msg package" ; return 1; }


rm message_package.zip -f
rm message_package.zip.* -f
rm msg_pack -r -f
wget 192.168.2.2:8082/build.msg/message_package.zip
mkdir msg_pack
cd msg_pack
unzip ../message_package.zip
cd ..
source msg_pack/install/local_setup.bash
