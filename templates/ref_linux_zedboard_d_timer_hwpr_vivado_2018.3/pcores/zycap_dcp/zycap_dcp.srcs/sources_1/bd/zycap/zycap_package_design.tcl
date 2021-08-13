#-----------------------
#Loading from existing component.
ipx::open_core {./component.xml}
#-----------------------

ipx::remove_all_port [ipx::current_core]
ipx::remove_all_file_group [ipx::current_core]
ipx::remove_all_bus_interface [ipx::current_core]

#-----------------------
# SYNTHESIS FILESET
#-----------------------
ipx::add_file_group {xilinx_verilogsynthesis} [ipx::current_core]
ipx::add_file hw_handoff/zycap.hwh [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]
ipx::add_file ip/zycap_axi_dma_0_0/zycap_axi_dma_0_0.xci [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]
ipx::add_file ip/zycap_icap_ctrl_0_1/zycap_icap_ctrl_0_1.xci [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]
ipx::add_file zycap_ooc.xdc [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]
ipx::add_file hdl/zycap.v [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]
set_property {model_name} {zycap} [ipx::get_file_group xilinx_verilogsynthesis [ipx::current_core]]

#-----------------------
# SIMULATION FILESET
#-----------------------
ipx::add_file_group {xilinx_verilogbehavioralsimulation} [ipx::current_core]
ipx::add_file hw_handoff/zycap.hwh [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]
ipx::add_file ip/zycap_axi_dma_0_0/zycap_axi_dma_0_0.xci [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]
ipx::add_file ip/zycap_icap_ctrl_0_1/zycap_icap_ctrl_0_1.xci [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]
ipx::add_file zycap_ooc.xdc [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]
ipx::add_file hdl/zycap.v [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]
set_property {model_name} {zycap} [ipx::get_file_group xilinx_verilogbehavioralsimulation [ipx::current_core]]

#-----------------------
# PORTS 
#-----------------------
ipx::add_ports_from_hdl [::ipx::current_core] -top_level_hdl_file ./hdl/zycap.v -top_module_name zycap

#-----------------------
# BUS INTERFACES 
#-----------------------
#------------------
#   Adding S_AXI_LITE
#------------------
ipx::add_bus_interface {S_AXI_LITE} [ipx::current_core]
set_property interface_mode {slave} [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]
set_property display_name {S_AXI_LITE} [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]

#   Adding Bus Type VNLV xilinx.com:interface:aximm:1.0
set_property {bus_type_vlnv} {xilinx.com:interface:aximm:1.0}  [ipx::get_bus_interface S_AXI_LITE [ipx::current_core]]

#   Adding Abstraction VNLV xilinx.com:interface:aximm_rtl:1.0
set_property {abstraction_type_vlnv} {xilinx.com:interface:aximm_rtl:1.0}  [ipx::get_bus_interface S_AXI_LITE [ipx::current_core]]

#   Adding PortMaps
set_property {physical_name} {S_AXI_LITE_araddr} [ipx::add_port_map {ARADDR}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_arready} [ipx::add_port_map {ARREADY}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_arvalid} [ipx::add_port_map {ARVALID}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_awaddr} [ipx::add_port_map {AWADDR}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_awready} [ipx::add_port_map {AWREADY}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_awvalid} [ipx::add_port_map {AWVALID}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_bready} [ipx::add_port_map {BREADY}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_bresp} [ipx::add_port_map {BRESP}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_bvalid} [ipx::add_port_map {BVALID}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_rdata} [ipx::add_port_map {RDATA}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_rready} [ipx::add_port_map {RREADY}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_rresp} [ipx::add_port_map {RRESP}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_rvalid} [ipx::add_port_map {RVALID}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_wdata} [ipx::add_port_map {WDATA}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_wready} [ipx::add_port_map {WREADY}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
set_property {physical_name} {S_AXI_LITE_wvalid} [ipx::add_port_map {WVALID}  [ipx::get_bus_interface {S_AXI_LITE} [ipx::current_core]]]
#------------------
#   Adding Parameters
#------------------
#   Adding M_AXI_MM2S
#------------------
ipx::add_bus_interface {M_AXI_MM2S} [ipx::current_core]
set_property interface_mode {master} [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]
set_property display_name {M_AXI_MM2S} [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]

#   Adding Bus Type VNLV xilinx.com:interface:aximm:1.0
set_property {bus_type_vlnv} {xilinx.com:interface:aximm:1.0}  [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]

#   Adding Abstraction VNLV xilinx.com:interface:aximm_rtl:1.0
set_property {abstraction_type_vlnv} {xilinx.com:interface:aximm_rtl:1.0}  [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]

#   Adding PortMaps
set_property {physical_name} {M_AXI_MM2S_araddr} [ipx::add_port_map {ARADDR}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arburst} [ipx::add_port_map {ARBURST}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arcache} [ipx::add_port_map {ARCACHE}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arlen} [ipx::add_port_map {ARLEN}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arprot} [ipx::add_port_map {ARPROT}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arready} [ipx::add_port_map {ARREADY}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arsize} [ipx::add_port_map {ARSIZE}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_arvalid} [ipx::add_port_map {ARVALID}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_rdata} [ipx::add_port_map {RDATA}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_rlast} [ipx::add_port_map {RLAST}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_rready} [ipx::add_port_map {RREADY}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_rresp} [ipx::add_port_map {RRESP}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
set_property {physical_name} {M_AXI_MM2S_rvalid} [ipx::add_port_map {RVALID}  [ipx::get_bus_interface {M_AXI_MM2S} [ipx::current_core]]]
#------------------
#   Adding Parameters
ipx::add_bus_parameter {SUPPORTS_NARROW_BURST}  [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]
set_property {value} {0} [ipx::get_bus_parameter {SUPPORTS_NARROW_BURST}   [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]]

ipx::add_bus_parameter {NUM_READ_OUTSTANDING}  [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]
set_property {value} {2} [ipx::get_bus_parameter {NUM_READ_OUTSTANDING}   [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]]

ipx::add_bus_parameter {MAX_BURST_LENGTH}  [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]
set_property {value} {256} [ipx::get_bus_parameter {MAX_BURST_LENGTH}   [ipx::get_bus_interface M_AXI_MM2S [ipx::current_core]]]

#------------------
#   Adding RST.axi_resetn
#------------------
ipx::add_bus_interface {RST.axi_resetn} [ipx::current_core]
set_property display_name {Reset} [ipx::get_bus_interface {RST.axi_resetn} [ipx::current_core]]
set_property interface_mode {slave} [ipx::get_bus_interface {RST.axi_resetn} [ipx::current_core]]

#   Adding Bus Type VNLV xilinx.com:signal:reset:1.0
set_property {bus_type_vlnv} {xilinx.com:signal:reset:1.0}  [ipx::get_bus_interface RST.axi_resetn [ipx::current_core]]

#   Adding Abstraction VNLV xilinx.com:signal:reset_rtl:1.0
set_property {abstraction_type_vlnv} {xilinx.com:signal:reset_rtl:1.0}  [ipx::get_bus_interface RST.axi_resetn [ipx::current_core]]

#   Adding PortMap
set_property {physical_name} {axi_resetn} [ipx::add_port_map {RST}  [ipx::get_bus_interface {RST.axi_resetn} [ipx::current_core]]]
#   Adding Parameters
ipx::add_bus_parameter {POLARITY}  [ipx::get_bus_interface RST.axi_resetn [ipx::current_core]]
set_property {value} {ACTIVE_LOW} [ipx::get_bus_parameter {POLARITY}   [ipx::get_bus_interface RST.axi_resetn [ipx::current_core]]]

#------------------
#   Adding CLK.s_axi_lite_aclk
#------------------
ipx::add_bus_interface {CLK.s_axi_lite_aclk} [ipx::current_core]
set_property display_name {Clk} [ipx::get_bus_interface {CLK.s_axi_lite_aclk} [ipx::current_core]]
set_property interface_mode {slave} [ipx::get_bus_interface {CLK.s_axi_lite_aclk} [ipx::current_core]]

#   Adding Bus Type VNLV xilinx.com:signal:clock:1.0
set_property {bus_type_vlnv} {xilinx.com:signal:clock:1.0}  [ipx::get_bus_interface CLK.s_axi_lite_aclk [ipx::current_core]]

#   Adding Abstraction VNLV xilinx.com:signal:clock_rtl:1.0
set_property {abstraction_type_vlnv} {xilinx.com:signal:clock_rtl:1.0}  [ipx::get_bus_interface CLK.s_axi_lite_aclk [ipx::current_core]]

#   Adding PortMap
set_property {physical_name} {s_axi_lite_aclk} [ipx::add_port_map {CLK}  [ipx::get_bus_interface {CLK.s_axi_lite_aclk} [ipx::current_core]]]
#   Adding Parameters
ipx::add_bus_parameter {ASSOCIATED_BUSIF}  [ipx::get_bus_interface CLK.s_axi_lite_aclk [ipx::current_core]]
set_property {value} {S_AXI_LITE:M_AXI_MM2S} [ipx::get_bus_parameter {ASSOCIATED_BUSIF}   [ipx::get_bus_interface CLK.s_axi_lite_aclk [ipx::current_core]]]

#------------------
#   Adding INT.mm2s_introut
#------------------
ipx::add_bus_interface {INT.mm2s_introut} [ipx::current_core]
set_property display_name {Interrupt} [ipx::get_bus_interface {INT.mm2s_introut} [ipx::current_core]]
set_property interface_mode {slave} [ipx::get_bus_interface {INT.mm2s_introut} [ipx::current_core]]

#   Adding Bus Type VNLV xilinx.com:signal:interrupt:1.0
set_property {bus_type_vlnv} {xilinx.com:signal:interrupt:1.0}  [ipx::get_bus_interface INT.mm2s_introut [ipx::current_core]]

#   Adding Abstraction VNLV xilinx.com:signal:interrupt_rtl:1.0
set_property {abstraction_type_vlnv} {xilinx.com:signal:interrupt_rtl:1.0}  [ipx::get_bus_interface INT.mm2s_introut [ipx::current_core]]

#   Adding PortMap
set_property {physical_name} {mm2s_introut} [ipx::add_port_map {INTERRUPT}  [ipx::get_bus_interface {INT.mm2s_introut} [ipx::current_core]]]
#   Adding Parameters
ipx::add_bus_parameter {SENSITIVITY}  [ipx::get_bus_interface INT.mm2s_introut [ipx::current_core]]
set_property {value} {LEVEL_HIGH} [ipx::get_bus_parameter {SENSITIVITY}   [ipx::get_bus_interface INT.mm2s_introut [ipx::current_core]]]


#-----------------------
# SAVE CORE TO REPOS
#-----------------------
ipx::create_default_gui_files [ipx::current_core]
ipx::save_core [ipx::current_core]
ipx::check_integrity  [ipx::current_core]
update_ip_catalog
