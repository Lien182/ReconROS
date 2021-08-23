-makelib xcelium_lib/xil_defaultlib -sv \
  "/opt/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
  "/opt/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv" \
  "/opt/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \
-endlib
-makelib xcelium_lib/xpm \
  "/opt/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_VCOMP.vhd" \
-endlib
-makelib xcelium_lib/lib_pkg_v1_0_2 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/0513/hdl/lib_pkg_v1_0_rfs.vhd" \
-endlib
-makelib xcelium_lib/fifo_generator_v13_2_3 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/64f4/simulation/fifo_generator_vlog_beh.v" \
-endlib
-makelib xcelium_lib/fifo_generator_v13_2_3 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/64f4/hdl/fifo_generator_v13_2_rfs.vhd" \
-endlib
-makelib xcelium_lib/fifo_generator_v13_2_3 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/64f4/hdl/fifo_generator_v13_2_rfs.v" \
-endlib
-makelib xcelium_lib/lib_fifo_v1_0_12 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/544a/hdl/lib_fifo_v1_0_rfs.vhd" \
-endlib
-makelib xcelium_lib/lib_srl_fifo_v1_0_2 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/51ce/hdl/lib_srl_fifo_v1_0_rfs.vhd" \
-endlib
-makelib xcelium_lib/lib_cdc_v1_0_2 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/ef1e/hdl/lib_cdc_v1_0_rfs.vhd" \
-endlib
-makelib xcelium_lib/axi_datamover_v5_1_20 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/dfb3/hdl/axi_datamover_v5_1_vh_rfs.vhd" \
-endlib
-makelib xcelium_lib/axi_sg_v4_1_11 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/efa7/hdl/axi_sg_v4_1_rfs.vhd" \
-endlib
-makelib xcelium_lib/axi_dma_v7_1_19 \
  "../../../../zycap_dcp.srcs/sources_1/bd/zycap/ipshared/09b0/hdl/axi_dma_v7_1_vh_rfs.vhd" \
-endlib
-makelib xcelium_lib/xil_defaultlib \
  "../../../bd/zycap/ip/zycap_axi_dma_0_0/sim/zycap_axi_dma_0_0.vhd" \
-endlib
-makelib xcelium_lib/xil_defaultlib \
  "../../../../zycap_dcp.srcs/sources_1/ipshared/cf83/verilog/icap_ctrl.v" \
  "../../../bd/zycap/ip/zycap_icap_ctrl_0_1/sim/zycap_icap_ctrl_0_1.v" \
  "../../../bd/zycap/hdl/zycap.v" \
  "../../../bd/zycap/sim/zycap.v" \
-endlib
-makelib xcelium_lib/xil_defaultlib \
  glbl.v
-endlib

