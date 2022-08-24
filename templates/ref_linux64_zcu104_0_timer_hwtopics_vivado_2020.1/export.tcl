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
#                 Felix Paul Jentzsch, UPB
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
        create_project -force $new_project_name $new_project_path -part xczu7ev-ffvc1156-2-e
    }


    # Save directory and project names to variables for easy reuse
    set proj_name [current_project]
    set proj_dir [get_property directory [current_project]]
    
    # Set project properties
    set_property "board_part" "xilinx.com:zcu104:1.1" $proj_name
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


    # Set 'sim_1' fileset properties
    set obj [get_filesets sim_1]
    set_property "transport_int_delay" "0" $obj
    set_property "transport_path_delay" "0" $obj
    set_property "xelab.nosort" "1" $obj
    set_property "xelab.unifast" "" $obj

    # Create 'synth_1' run (if not found)
    if {[string equal [get_runs -quiet synth_1] ""]} {
        create_run -name synth_1 -part xczu7ev-ffvc1156-2-e -flow {Vivado Synthesis 2016} -strategy "Vivado Synthesis Defaults" -constrset constrs_1
    } else {
        set_property strategy "Vivado Synthesis Defaults" [get_runs synth_1]
        set_property flow "Vivado Synthesis 2016" [get_runs synth_1]
    }

    # set the current synth run
    current_run -synthesis [get_runs synth_1]

    # Create 'impl_1' run (if not found)
    if {[string equal [get_runs -quiet impl_1] ""]} {
        create_run -name impl_1 -part xczu7ev-ffvc1156-2-e -flow {Vivado Implementation 2016} -strategy "Vivado Implementation Defaults" -constrset constrs_1 -parent_run synth_1
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

    # Add processing system for ZCU102 Board (Zynq UltraScale+ MPSoC) - also apply board preset settings
    create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.3 zynq_ultra_ps_e_0
    apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1" }  [get_bd_cells zynq_ultra_ps_e_0]
    
    # Make sure required AXI ports are active

    # Use HPC0 port instead of ACP
    #set_property -dict [list CONFIG.PSU__USE__S_AXI_ACP {1}] [get_bd_cells zynq_ultra_ps_e_0]
    set_property -dict [list CONFIG.PSU__USE__S_AXI_ACP {0}] [get_bd_cells zynq_ultra_ps_e_0]
    set_property -dict [list CONFIG.PSU__USE__S_AXI_GP0 {1}] [get_bd_cells zynq_ultra_ps_e_0]
    set_property -dict [list CONFIG.PSU__SAXIGP0__DATA_WIDTH {128}] [get_bd_cells zynq_ultra_ps_e_0]
    set_property -dict [list CONFIG.PSU__AFI0_COHERENCY {1}] [get_bd_cells zynq_ultra_ps_e_0]

    set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__MAXIGP0__DATA_WIDTH {64} CONFIG.PSU__USE__M_AXI_GP2 {0}] [get_bd_cells zynq_ultra_ps_e_0]
    set_property -dict [list CONFIG.PSU__USE__M_AXI_GP1 {0}] [get_bd_cells zynq_ultra_ps_e_0]

    # Enable high address fragmentation to allow access to upper 2GB of DRAM (enabled by default in newer Vivado versions)
    set_property -dict [list CONFIG.PSU__HIGH_ADDRESS__ENABLE {1}] [get_bd_cells zynq_ultra_ps_e_0]

    # Add interrupt port 
    set_property -dict [list CONFIG.PSU__USE__IRQ0 {1}] [get_bd_cells zynq_ultra_ps_e_0]

    # Set Frequencies
    set_property -dict [list CONFIG.PSU__CRL_APB__PL0_REF_CTRL__SRCSEL {IOPLL} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] [get_bd_cells zynq_ultra_ps_e_0]

    # Add AXI Busses and set properties
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_mem
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hwt
    set_property -dict [ list CONFIG.NUM_MI {1}  ] [get_bd_cells axi_mem]
    set_property -dict [ list CONFIG.NUM_MI {5}  ] [get_bd_cells axi_hwt]

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
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_memif_mmu_usp:1.0 reconos_memif_mmu_usp_0
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_osif_intc:1.0 reconos_osif_intc_0
    set_property -dict [list CONFIG.C_NUM_INTERRUPTS <<NUM_SLOTS>> ] [get_bd_cells reconos_osif_intc_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_osif:1.0 reconos_osif_0
    set_property -dict [list CONFIG.C_NUM_HWTS  <<NUM_SLOTS>> ] [get_bd_cells reconos_osif_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_proc_control:1.0 reconos_proc_control_0
    set_property -dict [list CONFIG.C_NUM_HWTS  <<NUM_SLOTS>> ] [get_bd_cells reconos_proc_control_0]
    
    create_bd_cell -type ip -vlnv cs.upb.de:reconos:timer:1.0 timer_0

	<<generate for SLOTS>>
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:<<HwtCoreName>>:[string range <<HwtCoreVersion>> 0 2] "slot_<<Id>>"
	
	<<end generate>>
        #"rt_sortdemo" { create_bd_cell -type ip -vlnv cs.upb.de:reconos:rt_sortdemo:1.0 "rt_sortdemo_$i" }
        
	<<generate for SLOTS(Async == "sync")>>
        # Add FIFOS between hardware threads and MEMIF and OSIF
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_sync:1.0 "reconos_fifo64_osif_hw2sw_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_sync:1.0 "reconos_fifo64_osif_sw2hw_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_sync:1.0 "reconos_fifo64_memif_hwt2mem_<<Id>>"
        create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_sync:1.0 "reconos_fifo64_memif_mem2hwt_<<Id>>"
	
	# Connect clock signals
	# FIFOs
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO64_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO64_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO64_Clk"]
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO64_Clk"]

	<<end generate>>

	<<generate for SLOTS(Async == "async")>>
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_async:1.0 "reconos_fifo64_osif_hw2sw_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_async:1.0 "reconos_fifo64_osif_sw2hw_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_async:1.0 "reconos_fifo64_memif_hwt2mem_<<Id>>"
	create_bd_cell -type ip -vlnv cs.upb.de:reconos:reconos_fifo64_async:1.0 "reconos_fifo64_memif_mem2hwt_<<Id>>"
	
	# Connect clock signals
	# FIFOs
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO64_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO64_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO64_S_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO64_S_Clk"]
	
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO64_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO64_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO64_M_Clk"]
	connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] [get_bd_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO64_M_Clk"]

	<<end generate>>

        # Add connections between FIFOs and other modules
	<<generate for SLOTS>>
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/OSIF_Hw2SW"] [get_bd_intf_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO64_M"]
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/OSIF_Sw2Hw"] [get_bd_intf_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO64_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO64_S"] [get_bd_intf_pins "reconos_osif_0/OSIF_hw2sw_<<Id>>"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO64_M"] [get_bd_intf_pins "reconos_osif_0/OSIF_sw2hw_<<Id>>"]
        connect_bd_net [get_bd_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO_Has_Data"] [get_bd_pins "reconos_osif_intc_0/OSIF_INTC_In_<<Id>>"]

        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/MEMIF64_Hwt2Mem"] [get_bd_intf_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO64_M"]
        connect_bd_intf_net [get_bd_intf_pins "slot_<<Id>>/MEMIF64_Mem2Hwt"] [get_bd_intf_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO64_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_memif_arbiter_0/MEMIF64_Hwt2Mem_<<Id>>"] [get_bd_intf_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO64_S"]
        connect_bd_intf_net [get_bd_intf_pins "reconos_memif_arbiter_0/MEMIF64_Mem2Hwt_<<Id>>"] [get_bd_intf_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO64_M"]
        
        # Set sizes of FIFOs
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {3}] [get_bd_cells "reconos_fifo64_osif_hw2sw_<<Id>>"]
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {3}] [get_bd_cells "reconos_fifo64_osif_sw2hw_<<Id>>"]
        
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {7}] [get_bd_cells "reconos_fifo64_memif_hwt2mem_<<Id>>"]
        set_property -dict [list CONFIG.C_FIFO_ADDR_WIDTH {7}] [get_bd_cells "reconos_fifo64_memif_mem2hwt_<<Id>>"]

        # HWTs
        connect_bd_net [get_bd_pins reconos_clock_0/CLK<<Clk>>_Out] [get_bd_pins "slot_<<Id>>/HWT_Clk"]

        # Resets
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "slot_<<Id>>/HWT_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo64_memif_mem2hwt_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo64_memif_hwt2mem_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo64_osif_hw2sw_<<Id>>/FIFO_Rst"]
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Rst_<<Id>>"] [get_bd_pins "reconos_fifo64_osif_sw2hw_<<Id>>/FIFO_Rst"]

	# Misc
        connect_bd_net [get_bd_pins "reconos_proc_control_0/PROC_Hwt_Signal_<<Id>>"] [get_bd_pins "slot_<<Id>>/HWT_Signal"]
	<<end generate>>


    #
    # Connections between components
    #

    # AXI
    connect_bd_intf_net -intf_net reconos_memif_memory_controller_0_M_AXI [get_bd_intf_pins reconos_memif_memory_controller_0/M00_AXI] [get_bd_intf_pins axi_mem/S00_AXI]
    connect_bd_intf_net -intf_net reconos_memif_memory_controller_0_S_AXI [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HPC0_FPD] [get_bd_intf_pins axi_mem/M00_AXI]

    connect_bd_intf_net -intf_net axi_hwt_S00_AXI [get_bd_intf_pins axi_hwt/S00_AXI] [get_bd_intf_pins zynq_ultra_ps_e_0/M_AXI_HPM0_FPD] 
    connect_bd_intf_net -intf_net axi_hwt_M00_AXI [get_bd_intf_pins axi_hwt/M00_AXI] [get_bd_intf_pins reconos_clock_0/S_AXI] 
    connect_bd_intf_net -intf_net axi_hwt_M01_AXI [get_bd_intf_pins axi_hwt/M01_AXI] [get_bd_intf_pins reconos_osif_intc_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M02_AXI [get_bd_intf_pins axi_hwt/M02_AXI] [get_bd_intf_pins reconos_osif_0/s00_axi]
    connect_bd_intf_net -intf_net axi_hwt_M03_AXI [get_bd_intf_pins axi_hwt/M03_AXI] [get_bd_intf_pins reconos_proc_control_0/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M04_AXI [get_bd_intf_pins axi_hwt/M04_AXI] [get_bd_intf_pins timer_0/S_AXI]

    # Memory controller
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_memory_controller_0/MEMIF64_Hwt2Mem_In] [get_bd_intf_pins reconos_memif_mmu_usp_0/MEMIF64_Hwt2Mem_Out]
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_memory_controller_0/MEMIF64_Mem2Hwt_In] [get_bd_intf_pins reconos_memif_mmu_usp_0/MEMIF64_Mem2Hwt_Out]

    # MMU
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_mmu_usp_0/MEMIF64_Hwt2Mem_In] [get_bd_intf_pins reconos_memif_arbiter_0/MEMIF64_Hwt2Mem_OUT]
    connect_bd_intf_net [get_bd_intf_pins reconos_memif_mmu_usp_0/MEMIF64_Mem2Hwt_In] [get_bd_intf_pins reconos_memif_arbiter_0/MEMIF64_Mem2Hwt_Out]
    connect_bd_net [get_bd_pins reconos_memif_mmu_usp_0/MMU_Pgf] [get_bd_pins reconos_proc_control_0/MMU_Pgf]
    connect_bd_net [get_bd_pins reconos_memif_mmu_usp_0/MMU_Retry] [get_bd_pins reconos_proc_control_0/MMU_Retry]
    connect_bd_net [get_bd_pins reconos_memif_mmu_usp_0/MMU_Pgd] [get_bd_pins reconos_proc_control_0/MMU_Pgd]
    connect_bd_net [get_bd_pins reconos_memif_mmu_usp_0/MMU_Fault_Addr] [get_bd_pins reconos_proc_control_0/MMU_Fault_Addr]
    set_property -dict [list CONFIG.C_TLB_SIZE {16}] [get_bd_cells reconos_memif_mmu_usp_0]

    #
    # Connect clocks - most clock inputs come from the reconos_clock module
    #
    connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins reconos_clock_0/CLK_Ref]

    connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Out] \
                            [get_bd_pins zynq_ultra_ps_e_0/maxihpm0_fpd_aclk] \
                            [get_bd_pins zynq_ultra_ps_e_0/saxihpc0_fpd_aclk] \
                            [get_bd_pins reconos_clock_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_memif_memory_controller_0/M00_AXI_ACLK] \
                            [get_bd_pins reconos_memif_mmu_usp_0/SYS_Clk] \
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
                            [get_bd_pins axi_hwt/S00_ACLK] \
                            [get_bd_pins reconos_osif_0/s00_axi_aclk] \
                            [get_bd_pins reconos_osif_intc_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_proc_control_0/S00_AXI_ACLK] \
                            [get_bd_pins reset_0/slowest_sync_clk] \
                            [get_bd_pins timer_0/S_AXI_ACLK]

    #
    # Connect Resets
    #
    connect_bd_net [get_bd_pins reconos_clock_0/CLK<<SYSCLK>>_Locked] [get_bd_pins reset_0/DCM_Locked] 

    connect_bd_net [get_bd_pins reset_0/ext_reset_in] [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] 
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
                            [get_bd_pins axi_hwt/S00_ARESETN]
    # Proc_control resets
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst] [get_bd_pins reconos_memif_arbiter_0/SYS_Rst]
    connect_bd_net [get_bd_pins reconos_memif_mmu_usp_0/SYS_Rst] [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst]

    # ReconoOS Peripherals reset by peripheral_aresetn
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_clock_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins timer_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_proc_control_0/S00_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_intc_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_0/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_memif_memory_controller_0/M00_AXI_ARESETN]

    #
    # Connect interrupts
    #
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_0
    set_property -dict [list CONFIG.CONST_VAL {0}] [get_bd_cells xlconstant_0]
    
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_0
    set_property -dict [list CONFIG.NUM_PORTS {8}] [get_bd_cells xlconcat_0]
    
    # This is needed to shift the interrupt lines to the right positions
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In0]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In1]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In2]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In3]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In4]
    connect_bd_net [get_bd_pins xlconstant_0/dout] [get_bd_pins xlconcat_0/In5]

    connect_bd_net [get_bd_pins reconos_osif_intc_0/OSIF_INTC_Out] [get_bd_pins xlconcat_0/In6]
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Pgf_Int] [get_bd_pins xlconcat_0/In7]
    connect_bd_net [get_bd_pins xlconcat_0/dout] [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq0]


    create_bd_cell -type ip -vlnv user.org:user:zycap:1.0 zycap_0
    set_property -dict [list CONFIG.NUM_MI {6}] [get_bd_cells axi_hwt]
    connect_bd_net [get_bd_pins zycap_0/axi_resetn] [get_bd_pins axi_hwt/M05_ARESETN]
    connect_bd_net [get_bd_pins axi_hwt/M05_ARESETN] [get_bd_pins reset_0/interconnect_aresetn]
    connect_bd_intf_net [get_bd_intf_pins zycap_0/S_AXI_LITE] -boundary_type upper [get_bd_intf_pins axi_hwt/M05_AXI]
    connect_bd_net [get_bd_pins zycap_0/s_axi_lite_aclk] [get_bd_pins axi_hwt/M05_ACLK]
    connect_bd_net [get_bd_pins axi_hwt/M05_ACLK] [get_bd_pins reconos_clock_0/CLK0_Out]
    set_property -dict [list CONFIG.PSU__USE__S_AXI_GP2 {1} CONFIG.PSU__SAXIGP2__DATA_WIDTH {32}] [get_bd_cells zynq_ultra_ps_e_0]
    connect_bd_intf_net [get_bd_intf_pins zycap_0/M_AXI_MM2S] [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HP0_FPD]
    connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/saxihp0_fpd_aclk] [get_bd_pins reconos_clock_0/CLK0_Out]




    #
    # Memory Map of peripherals
    #

    set_property -dict [list CONFIG.C_BASEADDR {0xA0000000} CONFIG.C_HIGHADDR {0xA000FFFF}] [get_bd_cells zycap_0]
    set_property -dict [list CONFIG.C_BASEADDR {0xA0130000} CONFIG.C_HIGHADDR {0xA013FFFF}] [get_bd_cells timer_0]
    set_property -dict [list CONFIG.C_BASEADDR {0xA0110000} CONFIG.C_HIGHADDR {0xA011FFFF}] [get_bd_cells reconos_osif_intc_0]
    set_property -dict [list CONFIG.C_BASEADDR {0xA0040000} CONFIG.C_HIGHADDR {0xA004FFFF}] [get_bd_cells reconos_clock_0]

    create_bd_addr_seg -range 0x80000000 -offset 0x00000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M00_AXI] [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP0/HPC0_DDR_LOW] SEG_zynq_ultra_ps_e_0_HPC0_DDR_LOW
    #create_bd_addr_seg -range 0x0800000000 -offset 0x0800000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M00_AXI] [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP0/HPC0_DDR_HIGH] SEG_zynq_ultra_ps_e_0_HPC0_DDR_HIGH
    create_bd_addr_seg -range 0x01000000 -offset 0xFF000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M00_AXI] [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP0/HPC0_LPS_OCM] SEG_zynq_ultra_ps_e_0_HPC0_LPS_OCM
    #create_bd_addr_seg -range 0x10000000 -offset 0xE0000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M00_AXI] [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP0/HPC0_PCIE_LOW] SEG_zynq_ultra_ps_e_0_HPC0_PCIE_LOW
    create_bd_addr_seg -range 0x20000000 -offset 0xC0000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M00_AXI] [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP0/HPC0_QSPI] SEG_zynq_ultra_ps_e_0_HPC0_QSPI
    create_bd_addr_seg -range 0x00010000 -offset 0xA0120000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs reconos_proc_control_0/S00_AXI/reg0] SEG_reconos_proc_control_0_S00_AXI_reg
    create_bd_addr_seg -range 0x00010000 -offset 0xA0040000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs reconos_clock_0/S_AXI/reg0] SEG_reconos_clock_0_reg0
    create_bd_addr_seg -range 0x00010000 -offset 0xA0100000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs reconos_osif_0/s00_axi/reg0] SEG_reconos_osif_0_reg0
    create_bd_addr_seg -range 0x00010000 -offset 0xA0110000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs reconos_osif_intc_0/S_AXI/reg0] SEG_reconos_osif_intc_0_reg0
    create_bd_addr_seg -range 0x00010000 -offset 0xA0130000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs timer_0/S_AXI/reg0] SEG_timer_0_reg0
    create_bd_addr_seg -range 0x00010000 -offset 0xA0000000 [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] [get_bd_addr_segs zycap_0/S_AXI_LITE/reg0] SEG_zycap_0_reg0


    assign_bd_address [get_bd_addr_segs {zynq_ultra_ps_e_0/SAXIGP2/HP0_DDR_LOW }]
    assign_bd_address [get_bd_addr_segs {zynq_ultra_ps_e_0/SAXIGP2/HP0_QSPI }]
    assign_bd_address [get_bd_addr_segs {zynq_ultra_ps_e_0/SAXIGP2/HP0_LPS_OCM }]

    # AXIS Interconnect for hwtopics
    <<generate for HWTOPICS>>
        startgroup
        create_bd_cell -type ip -vlnv xilinx.com:ip:axis_interconnect:2.1 axis_interconnect_<<Name>>
        endgroup
        #number of slave interfaces (publisher), number of master interfaces (subscriber)
        set_property -dict [list CONFIG.NUM_SI {<<NUM_PUBS>>} CONFIG.NUM_MI {<<NUM_SUBS>>} CONFIG.ARB_ON_TLAST {0}] [get_bd_cells axis_interconnect_<<Name>>]

        connect_bd_net [get_bd_pins axis_interconnect_<<Name>>/ACLK] [get_bd_pins reconos_clock_0/CLK1_Out]
        # connect publishers to slave interfaces of axis interconnect 
        <<=generate for PUBLISHERS=>>
            connect_bd_intf_net -boundary_type upper [get_bd_intf_pins axis_interconnect_<<Name>>/S0<<PubNr>>_AXIS] [get_bd_intf_pins slot_<<SlotId>>/<<Name>>]
            connect_bd_net [get_bd_pins axis_interconnect_<<Name>>/S0<<PubNr>>_AXIS_ACLK] [get_bd_pins reconos_clock_0/CLK1_Out]
            connect_bd_net [get_bd_pins axis_interconnect_<<Name>>/S0<<PubNr>>_AXIS_ARESETN] [get_bd_pins reset_0/interconnect_aresetn]
            
        <<=end generate=>>
        # connect subscribers to master interfaces of axis interconnect 
        <<=generate for SUBSCRIBERS=>>
            connect_bd_intf_net [get_bd_intf_pins slot_<<SlotId>>/<<Name>>] -boundary_type upper [get_bd_intf_pins axis_interconnect_<<Name>>/M0<<SubNr>>_AXIS]
            connect_bd_net [get_bd_pins axis_interconnect_<<Name>>/M0<<SubNr>>_AXIS_ACLK] [get_bd_pins reconos_clock_0/CLK1_Out]
            connect_bd_net [get_bd_pins axis_interconnect_<<Name>>/M0<<SubNr>>_AXIS_ARESETN] [get_bd_pins reset_0/interconnect_aresetn]
        <<=end generate=>>
    <<end generate>>

    # Update layout of block design
    regenerate_bd_layout

    #make wrapper file; vivado needs it to implement design
    make_wrapper -files [get_files $proj_dir/$proj_name.srcs/sources_1/bd/design_1/design_1.bd] -top
    add_files -norecurse $proj_dir/$proj_name.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.vhd
    update_compile_order -fileset sources_1
    update_compile_order -fileset sim_1
    set_property top design_1_wrapper [current_fileset]
	
	# Set BD generation mode to global (defaults to OOC only from Vivado 2016.3 onwards)
	set_property synth_checkpoint_mode None [get_files $proj_dir/$proj_name.srcs/sources_1/bd/design_1/design_1.bd]
	
  # Generate bitstream in .bin format (in addition to .bit)
  set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]

    save_bd_design
}

#
# MAIN
#

reconos_hw_setup $proj_name $proj_path $reconos_ip_dir
puts "\[RDK\]: Project creation finished."


