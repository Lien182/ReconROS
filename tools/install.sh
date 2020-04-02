#!/usr/bin/env bash
cat << "EOF"
#                                                        ____  _____
#                            ________  _________  ____  / __ \/ ___/
#                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
#                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
#                         /_/   \___/\___/\____/_/ /_/\____//____/
# 
# ======================================================================
# 
# Welcome to the ReconOS installation script
# It will help you setup a ReconOS working environment and a SD card to
# boot embedded Linux on the Zedboard. The script will download ~2.5 GB
# of data. This script was tested on Ubuntu 14.04.
# Refer to
# https://github.com/ReconOS/reconos.github.io/blob/develop/gettingstarted/tutorial/index.md
# for more information.
# 
#
EOF
# 
#   project:      ReconOS - Installation Script
#   author:       Georg Thombansen, Paderborn University
#   description:  Install ReconOS environment on local machine, prepare
#                 boot files for development board
# 
# ======================================================================

# Github tags for linux and u-boot
GITTAG="xilinx-v2016.2"

function pre_check {
  var=$3
    if command -v "$2" > /dev/null 2>&1; then
      printf "$1...AVAILABLE\n"
    else
      printf "$1...MISSING (ubuntu pkg: $4)\n"
      eval $var=false
    fi
}

function pre_check_lib {
    var=$3
    ldconfig -p | grep "$2" > /dev/null 2>&1
    if [ $? -eq 0 ]; then
      printf "$1...AVAILABLE\n"
    else
      printf "$1...MISSING (ubuntu pkg: $4)\n"
      eval $var=false
    fi
}

#
# [0] Argument parsing
#
if [ "$#" -ne 0 ]; then
  printf "Illegal number of parameters\n"
    printf "Usage: $0\n"
    exit 1
fi

#
# [1] check prerequisites
#
# python >= 3.4
PYVERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[1:2])))')
PYVERSION=$(($PYVERSION + 0))

printf "Checking prerequisites...\n"

if command -v python3 > /dev/null 2>&1 && [[ $PYVERSION -ge 4 ]]; then
  printf "Python...AVAILABLE\n"
else
  printf "Python...MISSING\n"
  ALLCHECK=false
fi

# more prerequisites
pre_check "Git" "git" ALLCHECK "git"
pre_check "Sed" "sed" ALLCHECK ""
pre_check "NFS" "nfsstat" ALLCHECK "nfs-kernel-server"
pre_check "Make" "make" ALLCHECK "build-essential"
pre_check_lib "lib32z1" "/usr/lib32/libz.so.1" ALLCHECK "lib32z1"
. /etc/lsb-release
if [[ $DISTRIB_RELEASE = "14.04" ]]; then
  pre_check_lib "libssl" "libssl.so$" ALLCHECK "libssl-dev"
fi

if [ "$ALLCHECK" = false ]; then
  printf "Please make sure that the prerequisites are met and run the install script again.\n"
  exit 1
fi

# Exit if [ $? -ne 0 ] from now on 
set -e

#
# [2] prepare working directory, setup ENV
#
printf "Type Directory name for ReconOS working directory:\n"

while true; do
  read -e -p "> " WD
  if [[ $WD = "" ]]; then
    continue
  fi
  WD=${WD/#\~/$HOME} # make tilde work
  if [ -d "$WD" ]; then # true if dir exists
    if find "$WD" -mindepth 1 -print -quit | grep -q .; then # true if dir not empty
      printf "The directory $(pwd)/$WD is not empty, please choose different name:\n"
    else # true if dir exists and emtpy
      break  
    fi;
  else # true if dir does not exist
    mkdir -p $WD
    break
  fi
done

WD=$( cd $WD ; pwd -P ) # change to absolute path
cd $WD

export ARCH=arm
printf "Type location of gnueabi compiler for cross compilation. It is shipped with Xilinx Vivado.\n"
printf "e.g. /opt/Xilinx/14.7/ISE_DS/EDK/gnu/arm/lin/bin/arm-xilinx-linux-gnueabi-gcc\n"
while true; do
  read -e -p "> " -i "/opt/Xilinx/SDK/2016.2/gnu/arm/lin/bin/arm-xilinx-linux-gnueabi-gcc" CROSS_COMPILE
  if [[ $CROSS_COMPILE = "" ]]; then
    continue
  fi
  CROSS_COMPILE=${CROSS_COMPILE/#\~/$HOME} # make tilde work
  if [[ ! -e "${CROSS_COMPILE}" ]]; then
    printf "$CROSS_COMPILE cannot be found, try again.\n"
    continue
  else
    break
  fi
done
CROSS_COMPILE_DIRNAME=$( cd $(dirname $CROSS_COMPILE) ; pwd -P ) # change to absolute path
CROSS_COMPILE_BASENAME=$(basename $CROSS_COMPILE)
CROSS_COMPILE="${CROSS_COMPILE_DIRNAME}/${CROSS_COMPILE_BASENAME}"
export CROSS_COMPILE=${CROSS_COMPILE%gcc}
export KDIR=$WD/linux-xlnx/
export PATH=$WD/u-boot-xlnx/tools:$PATH
export PATH=$WD/linux-xlnx/scripts/dtc:$PATH

#
# [3] prepare rootfs on NFS or ramdisk image
#
cat << "EOF"
This script supports two ways for setting up the root filesystem required by
the linux kernel:
1. NFS share: The root filesystem is stored in a folder on the host PC, which
   is exported via NFS. This method is the best option to quickly share files
   between Zedboard and Host PC, which increases comfort during development.
   Network connectivity via Ethernet is required.
2. Ramdisk image: Here the root filesystem is stored in a ramdisk image
   placed on the SD card. The Zedboard can work independently of a host PC
   but making changes to the filesystem and moving data to the Zedboard is
   more difficult.
EOF
printf "Type '1' for root filesystem as NFS share, '2' for Ramdisk image.\n"
while true; do
  read -e -p "> " -i "1" ROOTFS
  if [[ $ROOTFS = "" ]]; then
    continue
  fi
  if [[ $ROOTFS =~ ^[1-2]$ ]]; then
    break
  else
    printf "Not valid. Choose {1,2}.\n"
  fi
done

if [[ $ROOTFS = 1 ]]; then
  printf "Type board IP.\n"
  while true; do
    read -e -p "> " -i "192.168.0.1" BOARDIP
    if [[ $BOARDIP = "" ]]; then
      continue
    fi
    if [[ $BOARDIP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      break
    else
      printf "Not a valid IP, try again.\n"
    fi
  done

  # boardip = hostip + 1
  IFS='.' read -ra NBR <<< "$BOARDIP"
  INC=$((${NBR[3]} + 1))
  HOSTIPINC="${NBR[0]}.${NBR[1]}.${NBR[2]}.$INC"
  
  printf "Type host IP.\n"
  while true; do
    read -e -p "> " -i "$HOSTIPINC" HOSTIP
    if [[ $HOSTIP = "" ]]; then
      continue
    fi
    if [[ $HOSTIP =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      break
    else
      printf "Not a valid IP, try again.\n"
    fi
  done
fi

#
# [4] Check out repositories
#
git clone https://github.com/reconos/reconos
git clone git://git.busybox.net/busybox
git clone https://github.com/xilinx/u-boot-xlnx
git clone https://github.com/xilinx/linux-xlnx

#
# [5] Build scripts first (u-boot build requires dtc)
#
cd $WD/linux-xlnx
git checkout -b wb "$GITTAG"
make -j"$(nproc)" xilinx_zynq_defconfig
make -j"$(nproc)" prepare
make -j"$(nproc)" scripts


#
# [6] Build u-boot and tools (kernel build requires mkimage)
#
cd $WD/u-boot-xlnx
git checkout -b wb "$GITTAG"
if [[ $ROOTFS = 1 ]]; then
  sed -i '/.*sdboot=if mmcinfo;.*/{n;d}' $WD/u-boot-xlnx/include/configs/zynq-common.h
  sed -i '/.*load mmc 0 ${ramdisk_load_address} ${ramdisk_image} &&.*/{n;d}' $WD/u-boot-xlnx/include/configs/zynq-common.h
  sed -i 's/.*load mmc 0 ${ramdisk_load_address} ${ramdisk_image} &&.*/"bootm ${kernel_load_address} - ${devicetree_load_address}; " \\/' $WD/u-boot-xlnx/include/configs/zynq-common.h
  # sed -i 's|.*load mmc 0 ${ramdisk_load_address} ${ramdisk_image}.*|/* REMOVED */|' $WD/u-boot-xlnx/include/configs/zynq-common.h
  # sed -i 's|.*bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address};.*|"bootm ${kernel_load_address} - ${devicetree_load_address}; " \|' $WD/u-boot-xlnx/include/configs/zynq-common.h
else
  sed -i '/.*sdboot=if mmcinfo;.*/{n;d}' $WD/u-boot-xlnx/include/configs/zynq-common.h
fi

make -j"$(nproc)" zynq_zed_defconfig
make -j"$(nproc)"


#
# [7] Build kernel
#
cd $WD/linux-xlnx
if [[ $ROOTFS = 1 ]]; then
  BOOTARGS="    bootargs = \"console=ttyPS0,115200 root=/dev/nfs rw nfsroot=${HOSTIP}:${WD}/nfs,tcp,nfsvers=3 ip=${BOARDIP}:::255.255.255.0:reconos:eth0:off earlyprintk\";"
  sed -i "s|.*bootargs.*|$BOOTARGS|" ${WD}/linux-xlnx/arch/arm/boot/dts/zynq-zed.dts
  sed -i '/.*drv-vbus.*/a };\namba: amba {\nreconos_osif: reconos_osif@75a00000 {\ncompatible = "upb,reconos-osif-3.1";\nreg = <0x75a00000 0x10000>;\n};\nreconos_osif_intc: reconos_osif_intc@7b400000 {\ncompatible = "upb,reconos-osif-intc-3.1";\nreg = <0x7b400000 0x10000>;\ninterrup-parent = <&intc>;\ninterrupts = <0 58 4>;\n};\nreconos_proc_control: reconos_proc_control@6fe00000 {\ncompatible = "upb,reconos-control-3.1";\nreg = <0x6fe00000 0x10000>;\ninterrupt-parent = <&intc>;\ninterrupts = <0 59 4>;\n};' ${WD}/linux-xlnx/arch/arm/boot/dts/zynq-zed.dts
  # patch for develop-ic (not tested)
  # sed -i '/.*drv-vbus.*/a };\namba: amba {\nreconos_osif: reconos_osif@75a00000 {\ncompatible = "upb,reconos-osif-3.1";\nreg = <0x75a00000 0x10000>;\ninterrup-parent = <&intc>;\ninterrupts = <0 58 4>;\n};\nreconos_proc_control: reconos_proc_control@6fe00000 {\ncompatible = "upb,reconos-control-3.1";\nreg = <0x6fe00000 0x10000>;\ninterrupt-parent = <&intc>;\ninterrupts = <0 59 4>;\n};' ${WD}/linux-xlnx/arch/arm/boot/dts/zynq-zed.dts
else
  BOOTARGS="    bootargs = \"console=ttyPS0,115200 root=/dev/ram rw initrd=0x4000000 earlyprintk\";"
  sed -i "s|.*bootargs.*|$BOOTARGS|" ${WD}/linux-xlnx/arch/arm/boot/dts/zynq-zed.dts
  sed -i '/.*drv-vbus.*/a };\namba: amba {\nreconos_osif: reconos_osif@75a00000 {\ncompatible = "upb,reconos-osif-3.1";\nreg = <0x75a00000 0x10000>;\n};\nreconos_osif_intc: reconos_osif_intc@7b400000 {\ncompatible = "upb,reconos-osif-intc-3.1";\nreg = <0x7b400000 0x10000>;\ninterrup-parent = <&intc>;\ninterrupts = <0 58 4>;\n};\nreconos_proc_control: reconos_proc_control@6fe00000 {\ncompatible = "upb,reconos-control-3.1";\nreg = <0x6fe00000 0x10000>;\ninterrupt-parent = <&intc>;\ninterrupts = <0 59 4>;\n};' ${WD}/linux-xlnx/arch/arm/boot/dts/zynq-zed.dts
fi

make -j"$(nproc)" uImage LOADADDR=0x00008000
make -j"$(nproc)" dtbs

#
# [8] Build busybox
#
cd $WD/busybox
make -j"$(nproc)" defconfig

sed -i "s|.*CONFIG_STATIC.*|CONFIG_STATIC=y|" ${WD}/busybox/.config
make -j"$(nproc)"
make -j"$(nproc)" install

#
# [9] Prepare rootfs
#
mkdir $WD/nfs
cp -r _install/* $WD/nfs/

cd $WD/nfs/

mkdir dev etc etc/init.d lib mnt opt opt/reconos proc root sys tmp
cat > etc/inittab <<'EOF'
::sysinit:/etc/init.d/rcS

# Start an askfirst shell on the serial ports
ttyPS0::respawn:-/bin/sh

# What to do when restarting the init process
::restart:/sbin/init

# What to do before rebooting
::shutdown:/bin/umount -a -r
EOF

cat > etc/init.d/rcS <<'EOF'
#!/bin/sh

echo "Starting rcS..."

echo "++ Mounting filesystem"
mount -t proc none /proc
mount -t sysfs none /sys

echo "rcS Complete"
EOF

chmod +x etc/init.d/rcS
ln -s bin/busybox init

if [[ $ROOTFS = 1 ]]; then
  printf "Please provide root access to export NFS share!\n"
  sudo bash -c "cat >> /etc/exports << EOF
${WD}/nfs/ ${BOARDIP}(rw,no_subtree_check,all_squash,anonuid=$(id -u),anongid=$(id -g))
EOF"
  sudo exportfs -ar
fi

#
# [10] prepare sdcard
#
mkdir $WD/sdcard

cp $WD/u-boot-xlnx/spl/boot.bin $WD/sdcard/
cp $WD/u-boot-xlnx/u-boot.img $WD/sdcard/
cp $WD/linux-xlnx/arch/arm/boot/uImage $WD/sdcard/
cp $WD/linux-xlnx/arch/arm/boot/dts/zynq-zed.dtb $WD/sdcard/devicetree.dtb

#
# [11] Build ReconOS init files
#
cd $WD/reconos/linux/driver
git checkout develop
make -j"$(nproc)" RECONOS_ARCH=zynq RECONOS_OS=linux RECONOS_MMU=true PREFIX=$WD/nfs/opt/reconos install

#
# [12] Create ramdisk image
#
if [[ $ROOTFS = 2 ]]; then
  mkdir $WD/ramdisk
  cd $WD/ramdisk
  dd if=/dev/zero of=ramdisk.image bs=1024 count=16384
  mke2fs -F ramdisk.image -L "ramdisk" -b 1024 -m 0
  tune2fs ramdisk.image -i 0
  chmod a+rwx ramdisk.image
  mkdir $WD/ramdisk/mnt/
  printf "Please provide root access to mount ramdisk image!\n"
  sudo mount -o loop ramdisk.image $WD/ramdisk/mnt/
fi

#
# [13] move rootfs to ramdisk image
#
if [[ $ROOTFS = 2 ]]; then
  sudo cp -r $WD/nfs/* $WD/ramdisk/mnt/
  sudo umount $WD/ramdisk/mnt/
  gzip $WD/ramdisk/ramdisk.image
  mkimage -A arm -T ramdisk -C gzip -d $WD/ramdisk/ramdisk.image.gz $WD/ramdisk/uramdisk.image.gz
  cp $WD/ramdisk/uramdisk.image.gz $WD/sdcard/
fi

#
# [14] Final steps
#
printf "ReconOS has been installed successfully.\n\n"
printf "Please execute the following post-installation steps:\n"
printf "* Format a SD card as FAT32 with boot flag set.\n"
printf "* Copy content from ${WD}/sdcard/ to the SD card and insert to Zedboard.\n"
printf "* Set jumpers on Zedboard: MI02, MI03 and MI06 to GND, MI04 and MI05 to 3V3.\n"
if [[ $ROOTFS = 1 ]]; then
  printf "* Set the IP address of the host PC to ${HOSTIP}.\n"
fi
printf "* Turn on the board, connect with UART.\n"
printf "* If 'Zynq> ' prompt appears type 'boot' followd by 'Enter'.\n\n"
printf "The Linux prompt '#\ ' will appear.\n"
printf "Continue building the sort demo, or your own project.\n"
printf "Refer to https://github.com/ReconOS/reconos.github.io/blob/develop/gettingstarted/tutorial/index.md for more information.\n"
