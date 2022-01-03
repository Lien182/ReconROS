source /opt/Xilinx/Vivado/2020.1/settings64.sh
rm boot.bin -f
bootgen -image bootimage.bif -o boot.bin
