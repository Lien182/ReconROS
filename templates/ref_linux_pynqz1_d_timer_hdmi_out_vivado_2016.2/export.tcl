#                                                        ____  _____
#                            ________  _________  ____  / __ \/ ___/
#                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
#                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
#                         /_/   \___/\___/\____/_/ /_/\____//____/
# 
# ======================================================================
#
#   title:        ReconOS setup script for Vivado
#
#   project:      ReconOS
#   author:       Sebastian Meisner, University of Paderborn
#   description:  This TCL script sets up all modules and connections
#                 in an IP integrator block design needed to create
#                 a fully functional ReconoOS design.
#
# ======================================================================

<<reconos_preproc>>

variable script_file
set script_file "system.tcl"


# Help information for this script
proc help {} {
  variable script_file
  puts "\nDescription:"
  puts "This TCL script sets up all modules and connections in an IP integrator"
  puts "block design needed to create a fully functional ReconoOS design.\n"
  puts "Syntax when called in batch mode:"
  puts "vivado -mode tcl -source $script_file -tclargs \[-proj_name <Name> -proj_path <Path>\]" 
  puts "$script_file -tclargs \[--help\]\n"
  puts "Usage:"
  puts "Name                   Description"
  puts "-------------------------------------------------------------------------"
  puts "-proj_name <Name>        Optional: When given, a new preject will be"
  puts "                         created with the given name"
  puts "-proj_path <path>        Path to the newly created project"
  puts "\[--help\]               Print help information for this script"
  puts "-------------------------------------------------------------------------\n"
  exit 0
}


# Set the directory where the IP integrator cores live
set reconos_ip_dir [pwd]/pcores

set proj_name ""
set proj_path ""

# Parse command line arguments
if { $::argc > 0 } {
  for {set i 0} {$i < [llength $::argc]} {incr i} {
    set option [string trim [lindex $::argv $i]]
    switch -regexp -- $option {
      "-proj_name" { incr i; set proj_name  [lindex $::argv $i] }
      "-proj_path" { incr i; set proj_path  [lindex $::argv $i] }
      "-help"      { help }
      default {
        if { [regexp {^-} $option] } {
          puts "ERROR: Unknown option '$option' specified, please type '$script_file -tclargs --help' for usage info.\n"
          return 1
        }
      }
    }
  }
}


proc reconos_hw_delete {} {
    
    # get current project name and directory
    set proj_name [current_project]
    set proj_dir [get_property directory [current_project]]
    
    open_bd_design $proj_dir/$proj_name.srcs/sources_1/bd/design_1/design_1.bd
    remove_files $proj_dir/$proj_name.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.vhd
    file delete -force $proj_dir/$proj_name.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.vhd
    set_property source_mgmt_mode DisplayOnly [current_project]
    update_compile_order -fileset sim_1
    remove_files $proj_dir/$proj_name.srcs/sources_1/bd/design_1/design_1.bd
    file delete -force $proj_dir/$proj_name.srcs/sources_1/bd/design_1

}


proc reconos_hw_setup {new_project_name new_project_path reconos_ip_dir} {

    # Create new project if "new_project_name" is given.
    # Otherwise current project will be reused.
    if { [llength $new_project_name] > 0} {
        create_project -force $new_project_name $new_project_path -part xc7z020clg400-1
    }


    # Save directory and project names to variables for easy reuse
    set proj_name [current_project]
    set proj_dir [get_property directory [current_project]]
    
    # Set project properties
    #set_property "board_part" "em.avnet.com:zed:part0:1.3" $proj_name
    set_property "default_lib" "xil_defaultlib" $proj_name
    set_property "sim.ip.auto_export_scripts" "1" $proj_name
    set_property "simulator_language" "Mixed" $proj_name
    set_property "target_language" "VHDL" $proj_name

    # Create 'sources_1' fileset (if not found)
    if {[string equal [get_filesets -quiet sources_1] ""]} {
    create_fileset -srcset sources_1
    }


    # Create 'constrs_1' fileset (if not found)
    if {[string equal [get_filesets -quiet constrs_1] ""]} {
    create_fileset -constrset constrs_1
    }


    # Create 'sim_1' fileset (if not found)
    if {[string equal [get_filesets -quiet sim_1] ""]} {
    create_fileset -simset sim_1
    }

    add_files -fileset constrs_1 PYNQ-Z1_C.xdc


    # Set 'sim_1' fileset properties
    set obj [get_filesets sim_1]
    set_property "transport_int_delay" "0" $obj
    set_property "transport_path_delay" "0" $obj
    set_property "xelab.nosort" "1" $obj
    set_property "xelab.unifast" "" $obj

    # Create 'synth_1' run (if not found)
    if {[string equal [get_runs -quiet synth_1] ""]} {
        create_run -name synth_1 -part xc7z020clg400-1 -flow {Vivado Synthesis 2016} -strategy "Vivado Synthesis Defaults" -constrset constrs_1
    } else {
        set_property strategy "Vivado Synthesis Defaults" [get_runs synth_1]
        set_property flow "Vivado Synthesis 2016" [get_runs synth_1]
    }

    # set the current synth run
    current_run -synthesis [get_runs synth_1]

    # Create 'impl_1' run (if not found)
    if {[string equal [get_runs -quiet impl_1] ""]} {
        create_run -name impl_1 -part xc7z020clg400-1 -flow {Vivado Implementation 2016} -strategy "Vivado Implementation Defaults" -constrset constrs_1 -parent_run synth_1
    } else {
        set_property strategy "Vivado Implementation Defaults" [get_runs impl_1]
        set_property flow "Vivado Implementation 2016" [get_runs impl_1]
    }
    
    set obj [get_runs impl_1]
    set_property "steps.write_bitstream.args.readback_file" "0" $obj
    set_property "steps.write_bitstream.args.verbose" "0" $obj

    # set the current impl run
    current_run -implementation [get_runs impl_1]

    #
    # Start block design
    #
    create_bd_design "design_1"
    update_compile_order -fileset sources_1

    # Add reconos repository
    set_property  ip_repo_paths  $reconos_ip_dir [current_project]
    update_ip_catalog

	
    # Add system reset module
    create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset_0

    # Add processing system for Zynq Board
    create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 DDR
    create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 FIXED_IO
    create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0
    
    # Connect DDR and fixed IO
    connect_bd_intf_net -intf_net processing_system7_0_DDR [get_bd_intf_ports DDR] [get_bd_intf_pins processing_system7_0/DDR]
    connect_bd_intf_net -intf_net processing_system7_0_FIXED_IO [get_bd_intf_ports FIXED_IO] [get_bd_intf_pins processing_system7_0/FIXED_IO]
    
    # Make sure required AXI ports are active
    set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_USE_S_AXI_ACP {1}] [get_bd_cells processing_system7_0]
   
    # Add interrupt port 
    set_property -dict [list CONFIG.PCW_USE_FABRIC_INTERRUPT {1} CONFIG.PCW_IRQ_F2P_INTR {1}] [get_bd_cells processing_system7_0]
    
    # Set Frequencies
    set_property -dict [ list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} ] [get_bd_cells processing_system7_0]
    
    # Tie AxUSER signals on ACP port to '1' to enable cache coherancy
    set_property -dict [list CONFIG.PCW_USE_DEFAULT_ACP_USER_VAL {1}] [get_bd_cells processing_system7_0]

    # Add AXI Busses and set properties
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_mem
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hwt
    set_property -dict [ list CONFIG.NUM_MI {1}  ] [get_bd_cells axi_mem]
    set_property -dict [ list CONFIG.NUM_MI {7}  ] [get_bd_cells axi_hwt]

    # Add reconos stuff
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_clock:1.0 reconos_clock_0
    set_property -dict [list CONFIG.C_NUM_CLOCKS <<NUM_CLOCKS>>] [get_bd_cells reconos_clock_0]
    <<generate for CLOCKS>>
    set_property -dict [list CONFIG.C_CLK<<Id>>_CLKFBOUT_MULT <<M>>] [get_bd_cells reconos_clock_0]
    set_property -dict [list CONFIG.C_CLK<<Id>>_DIVCLK_DIVIDE 1    ] [get_bd_cells reconos_clock_0]
    set_property -dict [list CONFIG.C_CLK<<Id>>_CLKOUT_DIVIDE <<O>>] [get_bd_cells reconos_clock_0]
    <<end generate>>
    # Bugfix: literal for C_CLKIN_PERIOD has to be a real literal, e.g. needs to include the decimal point
    # Bugfix 2: Hmm, now vivado requests it to be an integer again....
    #set_property -dict [list CONFIG.C_CLKIN_PERIOD {10.0}] [get_bd_cells reconos_clock_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_memif_arbiter:1.0 reconos_memif_arbiter_0
    set_property -dict [list CONFIG.C_NUM_HWTS <<NUM_SLOTS>> ] [get_bd_cells reconos_memif_arbiter_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_memif_memory_controller:1.0 reconos_memif_memory_controller_0
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_memif_mmu_zynq:1.0 reconos_memif_mmu_zynq_0
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_osif_intc:1.0 reconos_osif_intc_0
    set_property -dict [list CONFIG.C_NUM_INTERRUPTS <<NUM_SLOTS>> ] [get_bd_cells reconos_osif_intc_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_osif:1.0 reconos_osif_0
    set_property -dict [list CONFIG.C_NUM_HWTS  <<NUM_SLOTS>> ] [get_bd_cells reconos_osif_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_proc_control:1.0 reconos_proc_control_0
    set_property -dict [list CONFIG.C_NUM_HWTS  <<NUM_SLOTS>> ] [get_bd_cells reconos_proc_control_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:timer:1.0 timer_0


   # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]


  # Create cell and set as current instance
  set hier_obj [create_bd_cell -type hier "pynq_hdmi_out"]
  current_bd_instance $hier_obj

  # Create interface pins
  create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 M00_AXI
  create_bd_intf_pin -mode Master -vlnv digilentinc.com:interface:tmds_rtl:1.0 TMDS
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 vdma_ctrl
  create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 vtc_ctrl

  # Create pins
  create_bd_pin -dir I -from 0 -to 0 axi4lite_aresetn
  create_bd_pin -dir I axi4lite_clk
  create_bd_pin -dir I -type clk axi4s_clk
  create_bd_pin -dir I axi4s_resetn
  create_bd_pin -dir I hdmio_clk

  # Create instance: axi_mem_intercon, and set properties
  set axi_mem_intercon [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_mem_intercon ]
  set_property -dict [ list \
CONFIG.NUM_MI {1} \
 ] $axi_mem_intercon

  # Create instance: axi_vdma_0, and set properties
  set axi_vdma_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma:6.3 axi_vdma_0 ]
  set_property -dict [ list \
CONFIG.c_enable_mm2s_delay_counter {0} \
CONFIG.c_enable_mm2s_frm_counter {0} \
CONFIG.c_enable_s2mm_delay_counter {0} \
CONFIG.c_enable_s2mm_frm_counter {0} \
CONFIG.c_include_mm2s_dre {1} \
CONFIG.c_include_s2mm {0} \
CONFIG.c_mm2s_genlock_mode {0} \
CONFIG.c_mm2s_linebuffer_depth {4096} \
CONFIG.c_s2mm_genlock_mode {0} \
 ] $axi_vdma_0

  # Create instance: axis_subset_converter_0, and set properties
  set axis_subset_converter_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axis_subset_converter:1.1 axis_subset_converter_0 ]
  set_property -dict [ list \
CONFIG.M_HAS_TKEEP {1} \
CONFIG.M_HAS_TLAST {1} \
CONFIG.M_TDATA_NUM_BYTES {3} \
CONFIG.M_TUSER_WIDTH {1} \
CONFIG.S_HAS_TKEEP {1} \
CONFIG.S_HAS_TLAST {1} \
CONFIG.S_TDATA_NUM_BYTES {4} \
CONFIG.S_TUSER_WIDTH {1} \
CONFIG.TDATA_REMAP {tdata[23:0]} \
CONFIG.TKEEP_REMAP {tkeep[2:0]} \
CONFIG.TLAST_REMAP {tlast[0]} \
CONFIG.TUSER_REMAP {tuser[0:0]} \
 ] $axis_subset_converter_0

  # Create instance: c_rsample_0, and set properties
  # Create instance: c_rsample_0, and set properties
  create_bd_cell -type ip -vlnv user.org:user:c_rsample:1.0 c_rsample_0
  
  # Create instance: gnd, and set properties
  set gnd [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 gnd ]
  set_property -dict [ list \
CONFIG.CONST_VAL {0} \
 ] $gnd

  # Create instance: proc_sys_reset, and set properties
  set proc_sys_reset [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset ]
  set_property -dict [ list \
CONFIG.C_AUX_RESET_HIGH {0} \
 ] $proc_sys_reset

  # Create instance: rgb2dvi_0, and set properties
  set rgb2dvi_0 [ create_bd_cell -type ip -vlnv digilentinc.com:ip:rgb2dvi:1.4 rgb2dvi_0 ]
  set_property -dict [ list \
CONFIG.kClkRange {3} \
CONFIG.kRstActiveHigh {false} \
 ] $rgb2dvi_0

  # Create instance: v_axi4s_vid_out_0, and set properties
  set v_axi4s_vid_out_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_axi4s_vid_out:4.0 v_axi4s_vid_out_0 ]
  set_property -dict [ list \
CONFIG.C_HAS_ASYNC_CLK {1} \
CONFIG.C_S_AXIS_VIDEO_FORMAT {0} \
CONFIG.C_VTG_MASTER_SLAVE {1} \
 ] $v_axi4s_vid_out_0

  # Create instance: v_rgb2ycrcb_0, and set properties
  set v_rgb2ycrcb_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_rgb2ycrcb:7.1 v_rgb2ycrcb_0 ]
  set_property -dict [ list \
CONFIG.ACTIVE_COLS {800} \
CONFIG.ACTIVE_ROWS {600} \
 ] $v_rgb2ycrcb_0

  # Create instance: v_tc_0, and set properties
  set v_tc_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:v_tc:6.1 v_tc_0 ]
  set_property -dict [ list \
CONFIG.GEN_F0_VBLANK_HEND {800} \
CONFIG.GEN_F0_VBLANK_HSTART {800} \
CONFIG.GEN_F0_VFRAME_SIZE {628} \
CONFIG.GEN_F0_VSYNC_HEND {800} \
CONFIG.GEN_F0_VSYNC_HSTART {800} \
CONFIG.GEN_F0_VSYNC_VEND {604} \
CONFIG.GEN_F0_VSYNC_VSTART {600} \
CONFIG.GEN_F1_VBLANK_HEND {800} \
CONFIG.GEN_F1_VBLANK_HSTART {800} \
CONFIG.GEN_F1_VFRAME_SIZE {628} \
CONFIG.GEN_F1_VSYNC_HEND {800} \
CONFIG.GEN_F1_VSYNC_HSTART {800} \
CONFIG.GEN_F1_VSYNC_VEND {604} \
CONFIG.GEN_F1_VSYNC_VSTART {600} \
CONFIG.GEN_HACTIVE_SIZE {800} \
CONFIG.GEN_HFRAME_SIZE {1056} \
CONFIG.GEN_HSYNC_END {968} \
CONFIG.GEN_HSYNC_START {840} \
CONFIG.GEN_VACTIVE_SIZE {600} \
CONFIG.VIDEO_MODE {800x600p} \
CONFIG.enable_detection {false} \
 ] $v_tc_0

  # Create instance: vcc, and set properties
  set vcc [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 vcc ]

  # Create interface connections
  connect_bd_intf_net -intf_net Conn1 [get_bd_intf_pins vtc_ctrl] [get_bd_intf_pins v_tc_0/ctrl]
  connect_bd_intf_net -intf_net Conn2 [get_bd_intf_pins M00_AXI] [get_bd_intf_pins axi_mem_intercon/M00_AXI]
  connect_bd_intf_net -intf_net Conn3 [get_bd_intf_pins vdma_ctrl] [get_bd_intf_pins axi_vdma_0/S_AXI_LITE]
  connect_bd_intf_net -intf_net Conn4 [get_bd_intf_pins TMDS] [get_bd_intf_pins rgb2dvi_0/TMDS]
  connect_bd_intf_net -intf_net axi_vdma_0_m_axi_mm2s [get_bd_intf_pins axi_mem_intercon/S00_AXI] [get_bd_intf_pins axi_vdma_0/M_AXI_MM2S]
  connect_bd_intf_net -intf_net axi_vdma_0_m_axis_mm2s [get_bd_intf_pins axi_vdma_0/M_AXIS_MM2S] [get_bd_intf_pins axis_subset_converter_0/S_AXIS]
  connect_bd_intf_net -intf_net axis_subset_converter_0_M_AXIS [get_bd_intf_pins axis_subset_converter_0/M_AXIS] [get_bd_intf_pins v_rgb2ycrcb_0/video_in]
  connect_bd_intf_net -intf_net c_rsample_0_m_axis_video [get_bd_intf_pins c_rsample_0/m_axis_video] [get_bd_intf_pins v_axi4s_vid_out_0/video_in]
  connect_bd_intf_net -intf_net v_axi4s_vid_out_0_vid_io_out [get_bd_intf_pins rgb2dvi_0/RGB] [get_bd_intf_pins v_axi4s_vid_out_0/vid_io_out]
  connect_bd_intf_net -intf_net v_rgb2ycrcb_0_video_out [get_bd_intf_pins c_rsample_0/s_axis_video] [get_bd_intf_pins v_rgb2ycrcb_0/video_out]
  connect_bd_intf_net -intf_net v_tc_0_vtiming_out [get_bd_intf_pins v_axi4s_vid_out_0/vtiming_in] [get_bd_intf_pins v_tc_0/vtiming_out]

  # Create port connections
  connect_bd_net -net aresetn_1 [get_bd_pins axi_mem_intercon/ARESETN] [get_bd_pins proc_sys_reset/interconnect_aresetn]
  connect_bd_net -net clk_1 [get_bd_pins hdmio_clk] [get_bd_pins rgb2dvi_0/PixelClk] [get_bd_pins v_axi4s_vid_out_0/vid_io_out_clk] [get_bd_pins v_tc_0/clk]
  connect_bd_net -net ext_reset_in_1 [get_bd_pins axi4s_resetn] [get_bd_pins proc_sys_reset/ext_reset_in]
  connect_bd_net -net gnd_const [get_bd_pins gnd/dout] [get_bd_pins v_axi4s_vid_out_0/vid_io_out_reset]
  connect_bd_net -net proc_sys_reset_peripheral_aresetn [get_bd_pins axi_mem_intercon/M00_ARESETN] [get_bd_pins axi_mem_intercon/S00_ARESETN] [get_bd_pins proc_sys_reset/peripheral_aresetn]
  connect_bd_net -net processing_system7_0_fclk_clk1 [get_bd_pins axi4s_clk] [get_bd_pins axi_mem_intercon/ACLK] [get_bd_pins axi_mem_intercon/M00_ACLK] [get_bd_pins axi_mem_intercon/S00_ACLK] [get_bd_pins axi_vdma_0/m_axi_mm2s_aclk] [get_bd_pins axi_vdma_0/m_axis_mm2s_aclk] [get_bd_pins axis_subset_converter_0/aclk] [get_bd_pins c_rsample_0/aclk] [get_bd_pins proc_sys_reset/slowest_sync_clk] [get_bd_pins v_axi4s_vid_out_0/aclk] [get_bd_pins v_rgb2ycrcb_0/aclk]
  connect_bd_net -net s_axi_aclk_1 [get_bd_pins axi4lite_clk] [get_bd_pins axi_vdma_0/s_axi_lite_aclk] [get_bd_pins v_tc_0/s_axi_aclk]
  connect_bd_net -net s_axi_aresetn_1 [get_bd_pins axi4lite_aresetn] [get_bd_pins v_tc_0/s_axi_aresetn]
  connect_bd_net -net v_axi4s_vid_out_0_vtg_ce [get_bd_pins v_axi4s_vid_out_0/vtg_ce] [get_bd_pins v_tc_0/gen_clken]
  connect_bd_net -net vcc_const [get_bd_pins axi_vdma_0/axi_resetn] [get_bd_pins axis_subset_converter_0/aresetn] [get_bd_pins c_rsample_0/aresetn] [get_bd_pins v_axi4s_vid_out_0/aclken] [get_bd_pins v_axi4s_vid_out_0/aresetn] [get_bd_pins v_axi4s_vid_out_0/vid_io_out_ce] [get_bd_pins v_rgb2ycrcb_0/aclken] [get_bd_pins v_rgb2ycrcb_0/aresetn] [get_bd_pins v_tc_0/clken] [get_bd_pins v_tc_0/resetn] [get_bd_pins v_tc_0/s_axi_aclken] [get_bd_pins vcc/dout]

  # Restore current instance
  current_bd_instance $oldCurInst



#############


	<<generate for SLOTS>>
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:<<HwtCoreName>>:[str range <<HwtCoreVersion>> 0 2] "slot_<<Id>>"
	
	<<end generate>>
        #"rt_sortdemo" { create_bd_cell -type ip -vlnv cs.upb.de:reconos:rt_sortdemo:1.0 "rt_sortdemo_$i" }
        
	<<generate for SLOTS(Async == "sync")>>
        # Add FIFOS between hardware threads and MEMIF and OSIF
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_sync:1.0 "reconos_fifo_osif_hw2sw_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_sync:1.0 "reconos_fifo_osif_sw2hw_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_sync:1.0 "reconos_fifo_memif_hwt2mem_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_sync:1.0 "reconos_fifo_memif_mem2hwt_<<Id>>"
	
	# Connect clock signals
	# FIFOs
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_Clk"]
	<<end generate>>
	
	<<generate for SLOTS(Async == "async")>>
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_async:1.0 "reconos_fifo_osif_hw2sw_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_async:1.0 "reconos_fifo_osif_sw2hw_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_async:1.0 "reconos_fifo_memif_hwt2mem_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo_async:1.0 "reconos_fifo_memif_mem2hwt_<<Id>>"
	
	# Connect clock signals
	# FIFOs
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_S_Clk"]
	
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_M_Clk"]
	<<end generate>>
	
        # Add connections between FIFOs and other modules
	<<generate for SLOTS>>
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/OSIF_Hw2SW"] [get_bd_intf_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_M"]
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/OSIF_Sw2Hw"] [get_bd_intf_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_S"] [get_bd_intf_pins "reconos_osif_0/OSIF_hw2sw_<<Id>>"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_M"] [get_bd_intf_pins "reconos_osif_0/OSIF_sw2hw_<<Id>>"]
        connect_bd_net [get_bd_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_Has_Data"] [get_bd_pins "reconos_osif_intc_0/OSIF_INTC_In_<<Id>>"]

        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/MEMIF_Hwt2Mem"] [get_bd_intf_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_M"]
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/MEMIF_Mem2Hwt"] [get_bd_intf_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_memif_arbiter_0/MEMIF_Hwt2Mem_<<Id>>"] [get_bd_intf_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_memif_arbiter_0/MEMIF_Mem2Hwt_<<Id>>"] [get_bd_intf_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_M"]
        
        # Set sizes of FIFOs
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {3}] [get_bd_cells "reconos_fifo_osif_hw2sw_<<Id>>"]
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {3}] [get_bd_cells "reconos_fifo_osif_sw2hw_<<Id>>"]
        
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {7}] [get_bd_cells "reconos_fifo_memif_hwt2mem_<<Id>>"]
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {7}] [get_bd_cells "reconos_fifo_memif_mem2hwt_<<Id>>"]

        # HWTs
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "slot_<<Id>>/HWT_Clk"]
	
        # Resets
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "slot_<<Id>>/HWT_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo_memif_mem2hwt_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo_memif_hwt2mem_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo_osif_hw2sw_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo_osif_sw2hw_<<Id>>/FIFO_Rst"]
	
	# Misc
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Signal_<<Id>>"] [get_bd_pins "slot_<<Id>>/HWT_Signal"]
	<<end generate>>


    #
    # Connections between components
    #

    # AXI
    connect_bd_intf_net -intf_net reconos_memif_memory_controller_0_M_AXI [get_bd_intf_pins reconos_memif_memory_controller_0/M_AXI] [get_bd_intf_pins axi_mem/S00_AXI]
    connect_bd_intf_net -intf_net reconos_memif_memory_controller_0_S_AXI [get_bd_intf_pins processing_system7_0/S_AXI_ACP] [get_bd_intf_pins axi_mem/M00_AXI]

    connect_bd_intf_net -intf_net axi_hwt_S00_AXI [get_bd_intf_pins axi_hwt/S00_AXI] [get_bd_intf_pins processing_system7_0/M_AXI_GP0] 
    connect_bd_intf_net -intf_net axi_hwt_M00_AXI [get_bd_intf_pins axi_hwt/M00_AXI] [get_bd_intf_pins reconos_clock_0/S_AXI] 
    connect_bd_intf_net -intf_net axi_hwt_M01_AXI [get_bd_intf_pins axi_hwt/M01_AXI] [get_bd_intf_pins reconos_osif_intc_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M02_AXI [get_bd_intf_pins axi_hwt/M02_AXI] [get_bd_intf_pins reconos_osif_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M03_AXI [get_bd_intf_pins axi_hwt/M03_AXI] [get_bd_intf_pins reconos_proc_control_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M04_AXI [get_bd_intf_pins axi_hwt/M04_AXI] [get_bd_intf_pins timer_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M05_AXI [get_bd_intf_pins axi_hwt/M05_AXI] [get_bd_intf_pins pynq_hdmi_out/vdma_ctrl]
    connect_bd_intf_net -intf_net axi_hwt_M06_AXI [get_bd_intf_pins axi_hwt/M06_AXI] [get_bd_intf_pins pynq_hdmi_out/vtc_ctrl]

    # Memory controller
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_memory_controller_0/MEMIF_Hwt2Mem_In] [get_bd_intf_pins reconos_memif_mmu_zynq_0/MEMIF_Hwt2Mem_Out]
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_memory_controller_0/MEMIF_Mem2Hwt_In] [get_bd_intf_pins reconos_memif_mmu_zynq_0/MEMIF_Mem2Hwt_Out]

    # MMU
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_mmu_zynq_0/MEMIF_Hwt2Mem_In] [get_bd_intf_pins reconos_memif_arbiter_0/MEMIF_Hwt2Mem_OUT]
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_mmu_zynq_0/MEMIF_Mem2Hwt_In] [get_bd_intf_pins reconos_memif_arbiter_0/MEMIF_Mem2Hwt_Out]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/MMU_Pgf] [get_bd_pins reconos_proc_control_0/MMU_Pgf]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/MMU_Retry] [get_bd_pins reconos_proc_control_0/MMU_Retry]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/MMU_Pgd] [get_bd_pins reconos_proc_control_0/MMU_Pgd]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/MMU_Fault_Addr] [get_bd_pins reconos_proc_control_0/MMU_Fault_Addr]
    set_property -dict [list CONFIG.C_TLB_SIZE {16}] [get_bd_cells reconos_memif_mmu_zynq_0]

    #
    # Connect clocks - most clock inputs come from the reconos_clock module
    #

    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins reconos_clock_0/CLK_Ref]

    connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] \
                            [get_bd_pins processing_system7_0/M_AXI_GP0_ACLK] \
                            [get_bd_pins processing_system7_0/S_AXI_ACP_ACLK] \
                            [get_bd_pins reconos_clock_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_memif_memory_controller_0/M_AXI_ACLK] \
                            [get_bd_pins reconos_memif_mmu_zynq_0/SYS_Clk] \
                            [get_bd_pins reconos_memif_arbiter_0/SYS_Clk] \
                            [get_bd_pins axi_mem/ACLK] \
                            [get_bd_pins axi_mem/M00_ACLK] \
                            [get_bd_pins axi_mem/S00_ACLK] \
                            [get_bd_pins axi_hwt/ACLK] \
                            [get_bd_pins axi_hwt/M00_ACLK] \
                            [get_bd_pins axi_hwt/M01_ACLK] \
                            [get_bd_pins axi_hwt/M02_ACLK] \
                            [get_bd_pins axi_hwt/M03_ACLK] \
                            [get_bd_pins axi_hwt/M04_ACLK] \
                            [get_bd_pins axi_hwt/M05_ACLK] \
                            [get_bd_pins axi_hwt/M06_ACLK] \
                            [get_bd_pins axi_hwt/S00_ACLK] \
                            [get_bd_pins reconos_osif_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_osif_intc_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_proc_control_0/S_AXI_ACLK] \
                            [get_bd_pins reset_0/slowest_sync_clk] \
                            [get_bd_pins timer_0/S_AXI_ACLK] \
                            [get_bd_pins pynq_hdmi_out/axi4lite_clk]
                            
    #
    # Connect Resets
    #
    connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Locked] [get_bd_pins reset_0/DCM_Locked] 

    connect_bd_net [get_bd_pins reset_0/ext_reset_in] [get_bd_pins processing_system7_0/FCLK_RESET0_N] 
    connect_bd_net [get_bd_pins reset_0/Interconnect_aresetn] \
                            [get_bd_pins axi_mem/ARESETN] \
                            [get_bd_pins axi_mem/M00_ARESETN] \
                            [get_bd_pins axi_mem/S00_ARESETN] \
                            [get_bd_pins axi_hwt/ARESETN] \
                            [get_bd_pins axi_hwt/M00_ARESETN] \
                            [get_bd_pins axi_hwt/M01_ARESETN] \
                            [get_bd_pins axi_hwt/M02_ARESETN] \
                            [get_bd_pins axi_hwt/M03_ARESETN] \
                            [get_bd_pins axi_hwt/M04_ARESETN] \
                            [get_bd_pins axi_hwt/M05_ARESETN] \
                            [get_bd_pins axi_hwt/M06_ARESETN] \
                            [get_bd_pins axi_hwt/S00_ARESETN] \
                            [get_bd_pins /pynq_hdmi_out/axi4lite_aresetn]

    # Proc_control resets
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst] [get_bd_pins reconos_memif_arbiter_0/SYS_Rst]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/SYS_Rst] [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst]

    # ReconoOS Peripherals reset by peripheral_aresetn
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_clock_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins timer_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_proc_control_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_intc_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_memif_memory_controller_0/M_AXI_ARESETN]

    #
    # Connect interrupts
    #
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_0
    set_property -dict [list CONFIG.CONST_VAL {0}] [get_bd_cells xlconstant_0]
    
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_0
    set_property -dict [list CONFIG.NUM_PORTS {16}] [get_bd_cells xlconcat_0]
    
    # This is needed to shift the interrupt lines to the right positions
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In0]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In1]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In2]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In3]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In4]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In5]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In6]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In7]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In8]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In9]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In10]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In11]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In12]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In13]
    
    connect_bd_net [get_bd_pins reconos_osif_intc_0/OSIF_INTC_Out] [get_bd_pins xlconcat_0/In14]
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Pgf_Int] [get_bd_pins xlconcat_0/In15]
    connect_bd_net [get_bd_pins xlconcat_0/dout] [get_bd_pins processing_system7_0/IRQ_F2P]


   make_bd_intf_pins_external  [get_bd_intf_pins pynq_hdmi_out/TMDS]

    #set_property name TMDS_Clk_p [get_bd_intf_ports TMDS_clk_p]
    #set_property name TMDS_Clk_n [get_bd_intf_ports TMDS_clk_n]
    #set_property name TMDS_Data_p [get_bd_intf_ports TMDS_data_p]
    #set_property name TMDS_Data_n [get_bd_intf_ports TMDS_data_n]

    set_property -dict [list CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {150}] [get_bd_cells processing_system7_0]
    set_property -dict [list CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {100}] [get_bd_cells processing_system7_0]
    
    create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:5.4 clk_wiz_0
    set_property -dict [list CONFIG.USE_LOCKED {false} CONFIG.USE_RESET {false} CONFIG.RESET_TYPE {ACTIVE_HIGH} CONFIG.RESET_PORT {reset}] [get_bd_cells clk_wiz_0]

    set_property -dict [list CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {80.000} CONFIG.MMCM_DIVCLK_DIVIDE {1} CONFIG.MMCM_CLKOUT0_DIVIDE_F {12.500} CONFIG.CLKOUT1_JITTER {137.143}] [get_bd_cells clk_wiz_0]
    set_property -dict [list CONFIG.PCW_EN_CLK1_PORT {1} CONFIG.PCW_EN_CLK2_PORT {1} CONFIG.PCW_EN_CLK3_PORT {1}] [get_bd_cells processing_system7_0]

   
    connect_bd_net [get_bd_pins clk_wiz_0/clk_in1] [get_bd_pins processing_system7_0/FCLK_CLK3]
    #connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins clk_wiz_0/resetn] 
    connect_bd_net [get_bd_pins clk_wiz_0/clk_out1] [get_bd_pins pynq_hdmi_out/hdmio_clk]
    set_property -dict [list CONFIG.PCW_USE_S_AXI_HP0 {1}] [get_bd_cells processing_system7_0]
    connect_bd_intf_net [get_bd_intf_pins processing_system7_0/S_AXI_HP0] -boundary_type upper [get_bd_intf_pins pynq_hdmi_out/M00_AXI]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK1] [get_bd_pins pynq_hdmi_out/axi4s_clk]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK1] [get_bd_pins processing_system7_0/S_AXI_HP0_ACLK]
  

    set_property -dict [list CONFIG.PCW_EN_RST1_PORT {1}] [get_bd_cells processing_system7_0]
    connect_bd_net [get_bd_pins pynq_hdmi_out/axi4s_resetn] [get_bd_pins processing_system7_0/FCLK_RESET1_N]
    #
    # Memory Map of peripheperals
    #

    set_property -dict [list CONFIG.C_BASEADDR {0x6fe00000} CONFIG.C_HIGHADDR {0x6fe0ffff}] [get_bd_cells reconos_proc_control_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x75a00000} CONFIG.C_HIGHADDR {0x75a0ffff}] [get_bd_cells reconos_osif_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x64a00000} CONFIG.C_HIGHADDR {0x64a0ffff}] [get_bd_cells timer_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x7b400000} CONFIG.C_HIGHADDR {0x7b40ffff}] [get_bd_cells reconos_osif_intc_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x69e00000} CONFIG.C_HIGHADDR {0x69e0ffff}] [get_bd_cells reconos_clock_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x43000000} CONFIG.C_HIGHADDR {0x4300FFFF}] [get_bd_cells pynq_hdmi_out/axi_vdma_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x43C00000} CONFIG.C_HIGHADDR {0x43C0ffff}] [get_bd_cells pynq_hdmi_out/v_tc_0]
    
    create_bd_addr_seg -range 64K -offset 0x6FE00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_proc_control_0/S_AXI/reg0 }] SEG1
    create_bd_addr_seg -range 64K -offset 0x75a00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_osif_0/S_AXI/reg0 }] SEG2
    create_bd_addr_seg -range 64K -offset 0x64a00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {timer_0/S_AXI/reg0 }] SEG3
    create_bd_addr_seg -range 64K -offset 0x7b400000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_osif_intc_0/S_AXI/reg0 }] SEG4
    create_bd_addr_seg -range 64K -offset 0x69e00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_clock_0/S_AXI/reg0 }] SEG5
    create_bd_addr_seg -range 64K -offset 0x43000000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {pynq_hdmi_out/axi_vdma_0/S_AXI_LITE/Reg }] SEG6
    create_bd_addr_seg -range 64K -offset 0x43C00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {pynq_hdmi_out/v_tc_0/ctrl/Reg }] SEG7

    assign_bd_address [get_bd_addr_segs {processing_system7_0/S_AXI_ACP/ACP_DDR_LOWOCM }]
    assign_bd_address [get_bd_addr_segs {processing_system7_0/S_AXI_ACP/ACP_M_AXI_GP0 }]
    assign_bd_address [get_bd_addr_segs {processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM }]
    
                            
    # Update layout of block design
    regenerate_bd_layout

    #make wrapper file; vivado needs it to implement design
    make_wrapper -files [get_files $proj_dir/$proj_name.srcs/sources_1/bd/design_1/design_1.bd] -top
    add_files -norecurse $proj_dir/$proj_name.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.vhd
    update_compile_order -fileset sources_1
    update_compile_order -fileset sim_1
    set_property top design_1_wrapper [current_fileset]
    save_bd_design
}

#
# MAIN
#

reconos_hw_setup $proj_name $proj_path $reconos_ip_dir
puts "\[RDK\]: Project creation finished."


