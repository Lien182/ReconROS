

rm design_1_wrapper.bit -f
rm filterdemo -f

wget 192.168.2.1:8082/build.hw/myReconOS.runs/impl_1/design_1_wrapper.bit
wget 192.168.2.1:8082/build.sw/filterdemo
cat design_1_wrapper.bit > /dev/xdevcfg

chmod +x filterdemo