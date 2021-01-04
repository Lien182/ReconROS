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
        create_project -force $new_project_name $new_project_path -part xc7z020clg484-1
    }


    # Save directory and project names to variables for easy reuse
    set proj_name [current_project]
    set proj_dir [get_property directory [current_project]]
    
    # Set project properties
    set_property "board_part" "em.avnet.com:zed:part0:1.3" $proj_name
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

    add_files -fileset constrs_1 zedboard_master.xdc

    # Set 'sim_1' fileset properties
    set obj [get_filesets sim_1]
    set_property "transport_int_delay" "0" $obj
    set_property "transport_path_delay" "0" $obj
    set_property "xelab.nosort" "1" $obj
    set_property "xelab.unifast" "" $obj

    # Create 'synth_1' run (if not found)
    if {[string equal [get_runs -quiet synth_1] ""]} {
        create_run -name synth_1 -part xc7z020clg484-1 -flow {Vivado Synthesis 2016} -strategy "Vivado Synthesis Defaults" -constrset constrs_1
    } else {
        set_property strategy "Vivado Synthesis Defaults" [get_runs synth_1]
        set_property flow "Vivado Synthesis 2016" [get_runs synth_1]
    }

    # set the current synth run
    current_run -synthesis [get_runs synth_1]

    # Create 'impl_1' run (if not found)
    if {[string equal [get_runs -quiet impl_1] ""]} {
        create_run -name impl_1 -part xc7z020clg484-1 -flow {Vivado Implementation 2016} -strategy "Vivado Implementation Defaults" -constrset constrs_1 -parent_run synth_1
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
    set_property -dict [ list CONFIG.NUM_MI {23}  ] [get_bd_cells axi_hwt]

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




    #bop_0

    create_bd_cell -type ip -vlnv cs.upb.de:bop:servo:1.0 servo_0

    create_bd_port -dir O JA1
    create_bd_port -dir O JA2
    create_bd_port -dir O JA3
    create_bd_port -dir O JA7
    create_bd_port -dir O JA8
    create_bd_port -dir O JA9 
    connect_bd_net [get_bd_ports JA1] [get_bd_pins servo_0/servo_0]
    connect_bd_net [get_bd_ports JA2] [get_bd_pins servo_0/servo_1]
    connect_bd_net [get_bd_ports JA3] [get_bd_pins servo_0/servo_2]
    connect_bd_net [get_bd_ports JA7] [get_bd_pins servo_0/servo_3]
    connect_bd_net [get_bd_ports JA8] [get_bd_pins servo_0/servo_4]
    connect_bd_net [get_bd_ports JA9] [get_bd_pins servo_0/servo_5]


    create_bd_cell -type ip -vlnv cs.upb.de:bop:touch:1.0 touch_0

    #Touch MISO
    create_bd_port -dir I JB3 
    connect_bd_net [get_bd_ports JB3] [get_bd_pins touch_0/TC_MISO]
    #Touch INT
    create_bd_port -dir I JB7
    connect_bd_net [get_bd_ports JB7] [get_bd_pins touch_0/TC_IRQ]
    #Touch SCLK
    create_bd_port -dir O JB1 
    connect_bd_net [get_bd_ports JB1] [get_bd_pins touch_0/TC_SCLK]
    #Touch MOSI
    create_bd_port -dir O JB2
    connect_bd_net [get_bd_ports JB2] [get_bd_pins touch_0/TC_MOSI]
    #Touch SSN
    create_bd_port -dir O JB4
    connect_bd_net [get_bd_ports JB4] [get_bd_pins touch_0/TC_SSn]


    #bop_1

    create_bd_cell -type ip -vlnv cs.upb.de:bop:servo:1.0 servo_1

    create_bd_port -dir O VGA_HS
    create_bd_port -dir O JA4
    create_bd_port -dir O JA10
    create_bd_port -dir O JB8
    create_bd_port -dir O JB9
    create_bd_port -dir O JB10 
    connect_bd_net [get_bd_ports VGA_HS] [get_bd_pins servo_1/servo_0]
    connect_bd_net [get_bd_ports JA4]  [get_bd_pins servo_1/servo_1]
    connect_bd_net [get_bd_ports JA10] [get_bd_pins servo_1/servo_2]
    connect_bd_net [get_bd_ports JB8]  [get_bd_pins servo_1/servo_3]
    connect_bd_net [get_bd_ports JB9]  [get_bd_pins servo_1/servo_4]
    connect_bd_net [get_bd_ports JB10] [get_bd_pins servo_1/servo_5]


    create_bd_cell -type ip -vlnv cs.upb.de:bop:touch:1.0 touch_1

    #Touch MISO
    create_bd_port -dir I JC3_N 
    connect_bd_net [get_bd_ports JC3_N] [get_bd_pins touch_1/TC_MISO]
    #Touch INT
    create_bd_port -dir I JC2_P
    connect_bd_net [get_bd_ports JC2_P] [get_bd_pins touch_1/TC_IRQ]
    #Touch SCLK
    create_bd_port -dir O JC1_P 
    connect_bd_net [get_bd_ports JC1_P] [get_bd_pins touch_1/TC_SCLK]
    #Touch MOSI
    create_bd_port -dir O JC3_P
    connect_bd_net [get_bd_ports JC3_P] [get_bd_pins touch_1/TC_MOSI]
    #Touch SSN
    create_bd_port -dir O JC1_N
    connect_bd_net [get_bd_ports JC1_N] [get_bd_pins touch_1/TC_SSn]


    #bop 2
    create_bd_cell -type ip -vlnv cs.upb.de:bop:servo:1.0 servo_2
    create_bd_port -dir O JD1_N
    create_bd_port -dir O JD3_N
    create_bd_port -dir O JD2_P
    create_bd_port -dir O JD4_P
    create_bd_port -dir O JD2_N
    create_bd_port -dir O JD4_N 
    connect_bd_net [get_bd_ports JD1_N] [get_bd_pins servo_2/servo_0]
    connect_bd_net [get_bd_ports JD3_N] [get_bd_pins servo_2/servo_1]
    connect_bd_net [get_bd_ports JD2_P] [get_bd_pins servo_2/servo_2]
    connect_bd_net [get_bd_ports JD4_P] [get_bd_pins servo_2/servo_3]
    connect_bd_net [get_bd_ports JD2_N] [get_bd_pins servo_2/servo_4]
    connect_bd_net [get_bd_ports JD4_N] [get_bd_pins servo_2/servo_5]


    create_bd_cell -type ip -vlnv cs.upb.de:bop:touch:1.0 touch_2

    #Touch MISO
    create_bd_port -dir I JD1_P 
    connect_bd_net [get_bd_ports JD1_P] [get_bd_pins touch_2/TC_MISO]
    #Touch INT
    create_bd_port -dir I JD3_P
    connect_bd_net [get_bd_ports JD3_P] [get_bd_pins touch_2/TC_IRQ]
    #Touch SCLK
    create_bd_port -dir O JC4_P 
    connect_bd_net [get_bd_ports JC4_P] [get_bd_pins touch_2/TC_SCLK]
    #Touch MOSI
    create_bd_port -dir O JC2_N
    connect_bd_net [get_bd_ports JC2_N] [get_bd_pins touch_2/TC_MOSI]
    #Touch SSN
    create_bd_port -dir O JC4_N
    connect_bd_net [get_bd_ports JC4_N] [get_bd_pins touch_2/TC_SSn]


    #AXI TIMER
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_timer:2.0 axi_timer_0
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_timer:2.0 axi_timer_1
    create_bd_cell -type ip -vlnv cs.upb.de:bop:difference_measurement_timer:1.0 difference_measurement_timer_0


    set IIC [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 IIC ]
    set IIC_1 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 IIC_1 ]
    set IIC_2 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:iic_rtl:1.0 IIC_2 ]
    set GPIO_BTN [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:gpio_rtl:1.0 GPIO_BTN ]

    set LD0 [ create_bd_port -dir O -from 0 -to 0 LD0 ]
    set LD1 [ create_bd_port -dir O LD1 ]
    set LD2 [ create_bd_port -dir O LD2 ]
    set LD3 [ create_bd_port -dir O LD3 ]
    set LD4 [ create_bd_port -dir O -from 0 -to 0 LD4 ]
    set LD5 [ create_bd_port -dir O LD5 ]
    set SW0 [ create_bd_port -dir I SW0 ]
    set hdmi_16_data [ create_bd_port -dir O -from 15 -to 0 hdmi_16_data ]
    set hdmi_16_data_e [ create_bd_port -dir O hdmi_16_data_e ]
    set hdmi_16_hsync [ create_bd_port -dir O hdmi_16_hsync ]
    set hdmi_16_vsync [ create_bd_port -dir O hdmi_16_vsync ]
    set hdmi_int [ create_bd_port -dir I -type intr hdmi_int ]
    set_property -dict [ list CONFIG.SENSITIVITY {LEVEL_HIGH}  ] $hdmi_int

    set hdmi_out_clk [ create_bd_port -dir O -type clk hdmi_out_clk ]
    set hdmi_rx_clk [ create_bd_port -dir I -type clk hdmi_rx_clk ]
    set hdmi_rx_data [ create_bd_port -dir I -from 15 -to 0 hdmi_rx_data ]

    # Create instance: axi_clkgen_0, and set properties
    set axi_clkgen_0 [ create_bd_cell -type ip -vlnv analog.com:user:axi_clkgen:1.0 axi_clkgen_0 ]

    # Create instance: axi_dmac_0, and set properties
    set axi_dmac_0 [ create_bd_cell -type ip -vlnv analog.com:user:axi_dmac:1.0 axi_dmac_0 ]
    set_property -dict [ list \
    CONFIG.CYCLIC {true} \
    CONFIG.DMA_2D_TRANSFER {true} \
    CONFIG.DMA_TYPE_DEST {1} \
    CONFIG.DMA_TYPE_SRC {0} \
    ] $axi_dmac_0

    # Create instance: axi_dmac_rx, and set properties
    set axi_dmac_rx [ create_bd_cell -type ip -vlnv analog.com:user:axi_dmac:1.0 axi_dmac_rx ]
    set_property -dict [ list \
    CONFIG.AXI_SLICE_DEST {true} \
    CONFIG.AXI_SLICE_SRC {true} \
    CONFIG.DMA_2D_TRANSFER {true} \
    CONFIG.DMA_LENGTH_WIDTH {14} \
    CONFIG.FIFO_SIZE {32} \
    CONFIG.MAX_BYTES_PER_BURST {2048} \
    CONFIG.SYNC_TRANSFER_START {true} \
    ] $axi_dmac_rx

    # Create instance: axi_gpio_0, and set properties
    set axi_gpio_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_gpio_0 ]
    set_property -dict [ list \
      CONFIG.C_ALL_INPUTS {1} \
      CONFIG.C_ALL_INPUTS_2 {1} \
      CONFIG.C_GPIO2_WIDTH {5} \
      CONFIG.C_GPIO_WIDTH {8} \
      CONFIG.C_INTERRUPT_PRESENT {1} \
      CONFIG.C_IS_DUAL {1} \
      CONFIG.GPIO2_BOARD_INTERFACE {btns_5bits} \
      CONFIG.GPIO_BOARD_INTERFACE {sws_8bits} \
    ] $axi_gpio_0

    # Create instance: axi_hdmi_rx_0, and set properties
    set axi_hdmi_rx_0 [ create_bd_cell -type ip -vlnv analog.com:user:axi_hdmi_rx:1.0 axi_hdmi_rx_0 ]
    set_property -dict [ list \
      CONFIG.IO_INTERFACE {1} \
    ] $axi_hdmi_rx_0

    # Create instance: axi_hdmi_tx_0, and set properties
    set axi_hdmi_tx_0 [ create_bd_cell -type ip -vlnv analog.com:user:axi_hdmi_tx:1.0 axi_hdmi_tx_0 ]

    # Create instance: axi_iic_ad7611, and set properties
    set axi_iic_ad7611 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic:2.0 axi_iic_ad7611 ]

    # Create instance: axi_interconnect_0, and set properties
    set axi_interconnect_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0 ]
    set_property -dict [ list \
      CONFIG.NUM_MI {1} \
    ] $axi_interconnect_0

    # Create instance: axi_interconnect_1, and set properties
    set axi_interconnect_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_1 ]
    set_property -dict [ list \
      CONFIG.NUM_MI {1} \
    ] $axi_interconnect_1

    # Create instance: gnd, and set properties
    set gnd [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 gnd ]
    set_property -dict [ list \
      CONFIG.CONST_VAL {0} \
    ] $gnd

    # Create instance: gnd_1, and set properties
    set gnd_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 gnd_1 ]
    set_property -dict [ list \
      CONFIG.CONST_VAL {0} \
    ] $gnd_1

    # Create instance: iic_fmc, and set properties
    set iic_fmc [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic:2.0 iic_fmc ]

    # Create instance: iic_main, and set properties
    set iic_main [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_iic:2.0 iic_main ]

      # Create instance: xlconcat_0, and set properties
  set xlconcat_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_0 ]
  set_property -dict [ list \
CONFIG.NUM_PORTS {16} \
 ] $xlconcat_0

  # Create instance: xlconcat_1, and set properties
  set xlconcat_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_1 ]
  set_property -dict [ list \
CONFIG.NUM_PORTS {11} \
 ] $xlconcat_1

  # Create instance: xlconstant_0, and set properties
  set xlconstant_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_0 ]
  set_property -dict [ list \
CONFIG.CONST_VAL {0} \
 ] $xlconstant_0

  # Create instance: xlconstant_1, and set properties
  set xlconstant_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 xlconstant_1 ]

  # Create instance: xlslice_0, and set properties
  set xlslice_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_0 ]
  set_property -dict [ list \
CONFIG.DIN_FROM {0} \
CONFIG.DIN_TO {0} \
CONFIG.DIN_WIDTH {16} \
 ] $xlslice_0

     set_property -dict [ list \
CONFIG.PCW_ACT_CAN_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_DCI_PERIPHERAL_FREQMHZ {10.158730} \
CONFIG.PCW_ACT_ENET0_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_ENET1_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ {100.000000} \
CONFIG.PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ {200.000000} \
CONFIG.PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ {200.000000} \
CONFIG.PCW_ACT_FPGA3_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_PCAP_PERIPHERAL_FREQMHZ {200.000000} \
CONFIG.PCW_ACT_QSPI_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_SDIO_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_SMC_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_SPI_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_ACT_TPIU_PERIPHERAL_FREQMHZ {200.000000} \
CONFIG.PCW_ACT_UART_PERIPHERAL_FREQMHZ {10.000000} \
CONFIG.PCW_APU_CLK_RATIO_ENABLE {6:2:1} \
CONFIG.PCW_APU_PERIPHERAL_FREQMHZ {666.666666} \
CONFIG.PCW_ARMPLL_CTRL_FBDIV {40} \
CONFIG.PCW_CAN0_CAN0_IO {<Select>} \
CONFIG.PCW_CAN0_GRP_CLK_ENABLE {0} \
CONFIG.PCW_CAN0_GRP_CLK_IO {<Select>} \
CONFIG.PCW_CAN0_PERIPHERAL_CLKSRC {External} \
CONFIG.PCW_CAN0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_CAN1_CAN1_IO {<Select>} \
CONFIG.PCW_CAN1_GRP_CLK_ENABLE {0} \
CONFIG.PCW_CAN1_GRP_CLK_IO {<Select>} \
CONFIG.PCW_CAN1_PERIPHERAL_CLKSRC {External} \
CONFIG.PCW_CAN1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_CAN_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_CAN_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_CAN_PERIPHERAL_DIVISOR1 {1} \
CONFIG.PCW_CAN_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_CLK0_FREQ {100000000} \
CONFIG.PCW_CLK1_FREQ {200000000} \
CONFIG.PCW_CLK2_FREQ {200000000} \
CONFIG.PCW_CLK3_FREQ {10000000} \
CONFIG.PCW_CPU_CPU_6X4X_MAX_RANGE {667} \
CONFIG.PCW_CPU_CPU_PLL_FREQMHZ {1333.333} \
CONFIG.PCW_CPU_PERIPHERAL_CLKSRC {ARM PLL} \
CONFIG.PCW_CPU_PERIPHERAL_DIVISOR0 {2} \
CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
CONFIG.PCW_DCI_PERIPHERAL_CLKSRC {DDR PLL} \
CONFIG.PCW_DCI_PERIPHERAL_DIVISOR0 {15} \
CONFIG.PCW_DCI_PERIPHERAL_DIVISOR1 {7} \
CONFIG.PCW_DCI_PERIPHERAL_FREQMHZ {10.159} \
CONFIG.PCW_DDRPLL_CTRL_FBDIV {32} \
CONFIG.PCW_DDR_DDR_PLL_FREQMHZ {1066.667} \
CONFIG.PCW_DDR_HPRLPR_QUEUE_PARTITION {HPR(0)/LPR(32)} \
CONFIG.PCW_DDR_HPR_TO_CRITICAL_PRIORITY_LEVEL {15} \
CONFIG.PCW_DDR_LPR_TO_CRITICAL_PRIORITY_LEVEL {2} \
CONFIG.PCW_DDR_PERIPHERAL_CLKSRC {DDR PLL} \
CONFIG.PCW_DDR_PERIPHERAL_DIVISOR0 {2} \
CONFIG.PCW_DDR_PORT0_HPR_ENABLE {0} \
CONFIG.PCW_DDR_PORT1_HPR_ENABLE {0} \
CONFIG.PCW_DDR_PORT2_HPR_ENABLE {0} \
CONFIG.PCW_DDR_PORT3_HPR_ENABLE {0} \
CONFIG.PCW_DDR_PRIORITY_READPORT_0 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_READPORT_1 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_READPORT_2 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_READPORT_3 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_0 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_1 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_2 {<Select>} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_3 {<Select>} \
CONFIG.PCW_DDR_RAM_HIGHADDR {0x1FFFFFFF} \
CONFIG.PCW_DDR_WRITE_TO_CRITICAL_PRIORITY_LEVEL {2} \
CONFIG.PCW_ENET0_ENET0_IO {<Select>} \
CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {0} \
CONFIG.PCW_ENET0_GRP_MDIO_IO {<Select>} \
CONFIG.PCW_ENET0_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_ENET0_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_ENET0_PERIPHERAL_DIVISOR1 {1} \
CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_ENET0_PERIPHERAL_FREQMHZ {1000 Mbps} \
CONFIG.PCW_ENET0_RESET_ENABLE {0} \
CONFIG.PCW_ENET0_RESET_IO {<Select>} \
CONFIG.PCW_ENET1_ENET1_IO {<Select>} \
CONFIG.PCW_ENET1_GRP_MDIO_ENABLE {0} \
CONFIG.PCW_ENET1_GRP_MDIO_IO {<Select>} \
CONFIG.PCW_ENET1_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_ENET1_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_ENET1_PERIPHERAL_DIVISOR1 {1} \
CONFIG.PCW_ENET1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_ENET1_PERIPHERAL_FREQMHZ {1000 Mbps} \
CONFIG.PCW_ENET1_RESET_ENABLE {0} \
CONFIG.PCW_ENET1_RESET_IO {<Select>} \
CONFIG.PCW_ENET_RESET_ENABLE {0} \
CONFIG.PCW_ENET_RESET_POLARITY {Active Low} \
CONFIG.PCW_ENET_RESET_SELECT {<Select>} \
CONFIG.PCW_EN_4K_TIMER {0} \
CONFIG.PCW_EN_CLK1_PORT {1} \
CONFIG.PCW_EN_CLK2_PORT {1} \
CONFIG.PCW_EN_EMIO_GPIO {1} \
CONFIG.PCW_FCLK0_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR0 {4} \
CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR1 {4} \
CONFIG.PCW_FCLK1_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK1_PERIPHERAL_DIVISOR0 {4} \
CONFIG.PCW_FCLK1_PERIPHERAL_DIVISOR1 {2} \
CONFIG.PCW_FCLK2_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK2_PERIPHERAL_DIVISOR0 {4} \
CONFIG.PCW_FCLK2_PERIPHERAL_DIVISOR1 {2} \
CONFIG.PCW_FCLK3_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK3_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_FCLK3_PERIPHERAL_DIVISOR1 {1} \
CONFIG.PCW_FCLK_CLK0_BUF {true} \
CONFIG.PCW_FCLK_CLK1_BUF {true} \
CONFIG.PCW_FCLK_CLK2_BUF {true} \
CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_FPGA3_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_FPGA_FCLK0_ENABLE {1} \
CONFIG.PCW_FPGA_FCLK1_ENABLE {1} \
CONFIG.PCW_FPGA_FCLK2_ENABLE {1} \
CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {1} \
CONFIG.PCW_GPIO_EMIO_GPIO_IO {12} \
CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH {12} \
CONFIG.PCW_GPIO_MIO_GPIO_ENABLE {1} \
CONFIG.PCW_GPIO_MIO_GPIO_IO {MIO} \
CONFIG.PCW_GPIO_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_I2C0_GRP_INT_ENABLE {0} \
CONFIG.PCW_I2C0_GRP_INT_IO {<Select>} \
CONFIG.PCW_I2C0_I2C0_IO {<Select>} \
CONFIG.PCW_I2C0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_I2C0_RESET_ENABLE {0} \
CONFIG.PCW_I2C0_RESET_IO {<Select>} \
CONFIG.PCW_I2C1_GRP_INT_ENABLE {0} \
CONFIG.PCW_I2C1_GRP_INT_IO {<Select>} \
CONFIG.PCW_I2C1_I2C1_IO {<Select>} \
CONFIG.PCW_I2C1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_I2C1_RESET_ENABLE {0} \
CONFIG.PCW_I2C1_RESET_IO {<Select>} \
CONFIG.PCW_I2C_PERIPHERAL_FREQMHZ {25} \
CONFIG.PCW_I2C_RESET_ENABLE {1} \
CONFIG.PCW_I2C_RESET_POLARITY {Active Low} \
CONFIG.PCW_I2C_RESET_SELECT {<Select>} \
CONFIG.PCW_IOPLL_CTRL_FBDIV {48} \
CONFIG.PCW_IO_IO_PLL_FREQMHZ {1600.000} \
CONFIG.PCW_IRQ_F2P_INTR {1} \
CONFIG.PCW_MIO_0_DIRECTION {inout} \
CONFIG.PCW_MIO_0_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_0_PULLUP {enabled} \
CONFIG.PCW_MIO_0_SLEW {slow} \
CONFIG.PCW_MIO_10_DIRECTION {inout} \
CONFIG.PCW_MIO_10_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_10_PULLUP {enabled} \
CONFIG.PCW_MIO_10_SLEW {slow} \
CONFIG.PCW_MIO_11_DIRECTION {inout} \
CONFIG.PCW_MIO_11_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_11_PULLUP {enabled} \
CONFIG.PCW_MIO_11_SLEW {slow} \
CONFIG.PCW_MIO_12_DIRECTION {inout} \
CONFIG.PCW_MIO_12_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_12_PULLUP {enabled} \
CONFIG.PCW_MIO_12_SLEW {slow} \
CONFIG.PCW_MIO_13_DIRECTION {inout} \
CONFIG.PCW_MIO_13_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_13_PULLUP {enabled} \
CONFIG.PCW_MIO_13_SLEW {slow} \
CONFIG.PCW_MIO_14_DIRECTION {inout} \
CONFIG.PCW_MIO_14_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_14_PULLUP {enabled} \
CONFIG.PCW_MIO_14_SLEW {slow} \
CONFIG.PCW_MIO_15_DIRECTION {inout} \
CONFIG.PCW_MIO_15_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_15_PULLUP {enabled} \
CONFIG.PCW_MIO_15_SLEW {slow} \
CONFIG.PCW_MIO_16_DIRECTION {inout} \
CONFIG.PCW_MIO_16_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_16_PULLUP {enabled} \
CONFIG.PCW_MIO_16_SLEW {slow} \
CONFIG.PCW_MIO_17_DIRECTION {inout} \
CONFIG.PCW_MIO_17_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_17_PULLUP {enabled} \
CONFIG.PCW_MIO_17_SLEW {slow} \
CONFIG.PCW_MIO_18_DIRECTION {inout} \
CONFIG.PCW_MIO_18_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_18_PULLUP {enabled} \
CONFIG.PCW_MIO_18_SLEW {slow} \
CONFIG.PCW_MIO_19_DIRECTION {inout} \
CONFIG.PCW_MIO_19_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_19_PULLUP {enabled} \
CONFIG.PCW_MIO_19_SLEW {slow} \
CONFIG.PCW_MIO_1_DIRECTION {inout} \
CONFIG.PCW_MIO_1_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_1_PULLUP {enabled} \
CONFIG.PCW_MIO_1_SLEW {slow} \
CONFIG.PCW_MIO_20_DIRECTION {inout} \
CONFIG.PCW_MIO_20_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_20_PULLUP {enabled} \
CONFIG.PCW_MIO_20_SLEW {slow} \
CONFIG.PCW_MIO_21_DIRECTION {inout} \
CONFIG.PCW_MIO_21_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_21_PULLUP {enabled} \
CONFIG.PCW_MIO_21_SLEW {slow} \
CONFIG.PCW_MIO_22_DIRECTION {inout} \
CONFIG.PCW_MIO_22_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_22_PULLUP {enabled} \
CONFIG.PCW_MIO_22_SLEW {slow} \
CONFIG.PCW_MIO_23_DIRECTION {inout} \
CONFIG.PCW_MIO_23_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_23_PULLUP {enabled} \
CONFIG.PCW_MIO_23_SLEW {slow} \
CONFIG.PCW_MIO_24_DIRECTION {inout} \
CONFIG.PCW_MIO_24_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_24_PULLUP {enabled} \
CONFIG.PCW_MIO_24_SLEW {slow} \
CONFIG.PCW_MIO_25_DIRECTION {inout} \
CONFIG.PCW_MIO_25_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_25_PULLUP {enabled} \
CONFIG.PCW_MIO_25_SLEW {slow} \
CONFIG.PCW_MIO_26_DIRECTION {inout} \
CONFIG.PCW_MIO_26_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_26_PULLUP {enabled} \
CONFIG.PCW_MIO_26_SLEW {slow} \
CONFIG.PCW_MIO_27_DIRECTION {inout} \
CONFIG.PCW_MIO_27_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_27_PULLUP {enabled} \
CONFIG.PCW_MIO_27_SLEW {slow} \
CONFIG.PCW_MIO_28_DIRECTION {inout} \
CONFIG.PCW_MIO_28_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_28_PULLUP {enabled} \
CONFIG.PCW_MIO_28_SLEW {slow} \
CONFIG.PCW_MIO_29_DIRECTION {inout} \
CONFIG.PCW_MIO_29_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_29_PULLUP {enabled} \
CONFIG.PCW_MIO_29_SLEW {slow} \
CONFIG.PCW_MIO_2_DIRECTION {inout} \
CONFIG.PCW_MIO_2_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_2_PULLUP {disabled} \
CONFIG.PCW_MIO_2_SLEW {slow} \
CONFIG.PCW_MIO_30_DIRECTION {inout} \
CONFIG.PCW_MIO_30_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_30_PULLUP {enabled} \
CONFIG.PCW_MIO_30_SLEW {slow} \
CONFIG.PCW_MIO_31_DIRECTION {inout} \
CONFIG.PCW_MIO_31_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_31_PULLUP {enabled} \
CONFIG.PCW_MIO_31_SLEW {slow} \
CONFIG.PCW_MIO_32_DIRECTION {inout} \
CONFIG.PCW_MIO_32_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_32_PULLUP {enabled} \
CONFIG.PCW_MIO_32_SLEW {slow} \
CONFIG.PCW_MIO_33_DIRECTION {inout} \
CONFIG.PCW_MIO_33_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_33_PULLUP {enabled} \
CONFIG.PCW_MIO_33_SLEW {slow} \
CONFIG.PCW_MIO_34_DIRECTION {inout} \
CONFIG.PCW_MIO_34_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_34_PULLUP {enabled} \
CONFIG.PCW_MIO_34_SLEW {slow} \
CONFIG.PCW_MIO_35_DIRECTION {inout} \
CONFIG.PCW_MIO_35_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_35_PULLUP {enabled} \
CONFIG.PCW_MIO_35_SLEW {slow} \
CONFIG.PCW_MIO_36_DIRECTION {inout} \
CONFIG.PCW_MIO_36_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_36_PULLUP {enabled} \
CONFIG.PCW_MIO_36_SLEW {slow} \
CONFIG.PCW_MIO_37_DIRECTION {inout} \
CONFIG.PCW_MIO_37_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_37_PULLUP {enabled} \
CONFIG.PCW_MIO_37_SLEW {slow} \
CONFIG.PCW_MIO_38_DIRECTION {inout} \
CONFIG.PCW_MIO_38_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_38_PULLUP {enabled} \
CONFIG.PCW_MIO_38_SLEW {slow} \
CONFIG.PCW_MIO_39_DIRECTION {inout} \
CONFIG.PCW_MIO_39_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_39_PULLUP {enabled} \
CONFIG.PCW_MIO_39_SLEW {slow} \
CONFIG.PCW_MIO_3_DIRECTION {inout} \
CONFIG.PCW_MIO_3_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_3_PULLUP {disabled} \
CONFIG.PCW_MIO_3_SLEW {slow} \
CONFIG.PCW_MIO_40_DIRECTION {inout} \
CONFIG.PCW_MIO_40_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_40_PULLUP {enabled} \
CONFIG.PCW_MIO_40_SLEW {slow} \
CONFIG.PCW_MIO_41_DIRECTION {inout} \
CONFIG.PCW_MIO_41_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_41_PULLUP {enabled} \
CONFIG.PCW_MIO_41_SLEW {slow} \
CONFIG.PCW_MIO_42_DIRECTION {inout} \
CONFIG.PCW_MIO_42_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_42_PULLUP {enabled} \
CONFIG.PCW_MIO_42_SLEW {slow} \
CONFIG.PCW_MIO_43_DIRECTION {inout} \
CONFIG.PCW_MIO_43_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_43_PULLUP {enabled} \
CONFIG.PCW_MIO_43_SLEW {slow} \
CONFIG.PCW_MIO_44_DIRECTION {inout} \
CONFIG.PCW_MIO_44_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_44_PULLUP {enabled} \
CONFIG.PCW_MIO_44_SLEW {slow} \
CONFIG.PCW_MIO_45_DIRECTION {inout} \
CONFIG.PCW_MIO_45_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_45_PULLUP {enabled} \
CONFIG.PCW_MIO_45_SLEW {slow} \
CONFIG.PCW_MIO_46_DIRECTION {inout} \
CONFIG.PCW_MIO_46_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_46_PULLUP {enabled} \
CONFIG.PCW_MIO_46_SLEW {slow} \
CONFIG.PCW_MIO_47_DIRECTION {inout} \
CONFIG.PCW_MIO_47_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_47_PULLUP {enabled} \
CONFIG.PCW_MIO_47_SLEW {slow} \
CONFIG.PCW_MIO_48_DIRECTION {inout} \
CONFIG.PCW_MIO_48_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_48_PULLUP {enabled} \
CONFIG.PCW_MIO_48_SLEW {slow} \
CONFIG.PCW_MIO_49_DIRECTION {inout} \
CONFIG.PCW_MIO_49_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_49_PULLUP {enabled} \
CONFIG.PCW_MIO_49_SLEW {slow} \
CONFIG.PCW_MIO_4_DIRECTION {inout} \
CONFIG.PCW_MIO_4_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_4_PULLUP {disabled} \
CONFIG.PCW_MIO_4_SLEW {slow} \
CONFIG.PCW_MIO_50_DIRECTION {inout} \
CONFIG.PCW_MIO_50_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_50_PULLUP {enabled} \
CONFIG.PCW_MIO_50_SLEW {slow} \
CONFIG.PCW_MIO_51_DIRECTION {inout} \
CONFIG.PCW_MIO_51_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_51_PULLUP {enabled} \
CONFIG.PCW_MIO_51_SLEW {slow} \
CONFIG.PCW_MIO_52_DIRECTION {inout} \
CONFIG.PCW_MIO_52_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_52_PULLUP {enabled} \
CONFIG.PCW_MIO_52_SLEW {slow} \
CONFIG.PCW_MIO_53_DIRECTION {inout} \
CONFIG.PCW_MIO_53_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_53_PULLUP {enabled} \
CONFIG.PCW_MIO_53_SLEW {slow} \
CONFIG.PCW_MIO_5_DIRECTION {inout} \
CONFIG.PCW_MIO_5_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_5_PULLUP {disabled} \
CONFIG.PCW_MIO_5_SLEW {slow} \
CONFIG.PCW_MIO_6_DIRECTION {inout} \
CONFIG.PCW_MIO_6_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_6_PULLUP {disabled} \
CONFIG.PCW_MIO_6_SLEW {slow} \
CONFIG.PCW_MIO_7_DIRECTION {out} \
CONFIG.PCW_MIO_7_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_7_PULLUP {disabled} \
CONFIG.PCW_MIO_7_SLEW {slow} \
CONFIG.PCW_MIO_8_DIRECTION {out} \
CONFIG.PCW_MIO_8_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_8_PULLUP {disabled} \
CONFIG.PCW_MIO_8_SLEW {slow} \
CONFIG.PCW_MIO_9_DIRECTION {inout} \
CONFIG.PCW_MIO_9_IOTYPE {LVCMOS 3.3V} \
CONFIG.PCW_MIO_9_PULLUP {enabled} \
CONFIG.PCW_MIO_9_SLEW {slow} \
CONFIG.PCW_MIO_TREE_PERIPHERALS {GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO#GPIO} \
CONFIG.PCW_MIO_TREE_SIGNALS {gpio[0]#gpio[1]#gpio[2]#gpio[3]#gpio[4]#gpio[5]#gpio[6]#gpio[7]#gpio[8]#gpio[9]#gpio[10]#gpio[11]#gpio[12]#gpio[13]#gpio[14]#gpio[15]#gpio[16]#gpio[17]#gpio[18]#gpio[19]#gpio[20]#gpio[21]#gpio[22]#gpio[23]#gpio[24]#gpio[25]#gpio[26]#gpio[27]#gpio[28]#gpio[29]#gpio[30]#gpio[31]#gpio[32]#gpio[33]#gpio[34]#gpio[35]#gpio[36]#gpio[37]#gpio[38]#gpio[39]#gpio[40]#gpio[41]#gpio[42]#gpio[43]#gpio[44]#gpio[45]#gpio[46]#gpio[47]#gpio[48]#gpio[49]#gpio[50]#gpio[51]#gpio[52]#gpio[53]} \
CONFIG.PCW_NAND_CYCLES_T_AR {1} \
CONFIG.PCW_NAND_CYCLES_T_CLR {1} \
CONFIG.PCW_NAND_CYCLES_T_RC {11} \
CONFIG.PCW_NAND_CYCLES_T_REA {1} \
CONFIG.PCW_NAND_CYCLES_T_RR {1} \
CONFIG.PCW_NAND_CYCLES_T_WC {11} \
CONFIG.PCW_NAND_CYCLES_T_WP {1} \
CONFIG.PCW_NAND_GRP_D8_ENABLE {0} \
CONFIG.PCW_NAND_GRP_D8_IO {<Select>} \
CONFIG.PCW_NAND_NAND_IO {<Select>} \
CONFIG.PCW_NAND_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_NOR_CS0_T_CEOE {1} \
CONFIG.PCW_NOR_CS0_T_PC {1} \
CONFIG.PCW_NOR_CS0_T_RC {11} \
CONFIG.PCW_NOR_CS0_T_TR {1} \
CONFIG.PCW_NOR_CS0_T_WC {11} \
CONFIG.PCW_NOR_CS0_T_WP {1} \
CONFIG.PCW_NOR_CS0_WE_TIME {0} \
CONFIG.PCW_NOR_CS1_T_CEOE {1} \
CONFIG.PCW_NOR_CS1_T_PC {1} \
CONFIG.PCW_NOR_CS1_T_RC {11} \
CONFIG.PCW_NOR_CS1_T_TR {1} \
CONFIG.PCW_NOR_CS1_T_WC {11} \
CONFIG.PCW_NOR_CS1_T_WP {1} \
CONFIG.PCW_NOR_CS1_WE_TIME {0} \
CONFIG.PCW_NOR_GRP_A25_ENABLE {0} \
CONFIG.PCW_NOR_GRP_A25_IO {<Select>} \
CONFIG.PCW_NOR_GRP_CS0_ENABLE {0} \
CONFIG.PCW_NOR_GRP_CS0_IO {<Select>} \
CONFIG.PCW_NOR_GRP_CS1_ENABLE {0} \
CONFIG.PCW_NOR_GRP_CS1_IO {<Select>} \
CONFIG.PCW_NOR_GRP_SRAM_CS0_ENABLE {0} \
CONFIG.PCW_NOR_GRP_SRAM_CS0_IO {<Select>} \
CONFIG.PCW_NOR_GRP_SRAM_CS1_ENABLE {0} \
CONFIG.PCW_NOR_GRP_SRAM_CS1_IO {<Select>} \
CONFIG.PCW_NOR_GRP_SRAM_INT_ENABLE {0} \
CONFIG.PCW_NOR_GRP_SRAM_INT_IO {<Select>} \
CONFIG.PCW_NOR_NOR_IO {<Select>} \
CONFIG.PCW_NOR_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_NOR_SRAM_CS0_T_CEOE {1} \
CONFIG.PCW_NOR_SRAM_CS0_T_PC {1} \
CONFIG.PCW_NOR_SRAM_CS0_T_RC {11} \
CONFIG.PCW_NOR_SRAM_CS0_T_TR {1} \
CONFIG.PCW_NOR_SRAM_CS0_T_WC {11} \
CONFIG.PCW_NOR_SRAM_CS0_T_WP {1} \
CONFIG.PCW_NOR_SRAM_CS0_WE_TIME {0} \
CONFIG.PCW_NOR_SRAM_CS1_T_CEOE {1} \
CONFIG.PCW_NOR_SRAM_CS1_T_PC {1} \
CONFIG.PCW_NOR_SRAM_CS1_T_RC {11} \
CONFIG.PCW_NOR_SRAM_CS1_T_TR {1} \
CONFIG.PCW_NOR_SRAM_CS1_T_WC {11} \
CONFIG.PCW_NOR_SRAM_CS1_T_WP {1} \
CONFIG.PCW_NOR_SRAM_CS1_WE_TIME {0} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY0 {0.063} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY1 {0.062} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY2 {0.065} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY3 {0.083} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_0 {-0.007} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_1 {-0.010} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_2 {-0.006} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_3 {-0.048} \
CONFIG.PCW_PCAP_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_PCAP_PERIPHERAL_DIVISOR0 {8} \
CONFIG.PCW_PCAP_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_PERIPHERAL_BOARD_PRESET {part0} \
CONFIG.PCW_PJTAG_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_PJTAG_PJTAG_IO {<Select>} \
CONFIG.PCW_PLL_BYPASSMODE_ENABLE {0} \
CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3V} \
CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 3.3V} \
CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE {0} \
CONFIG.PCW_QSPI_GRP_FBCLK_IO {<Select>} \
CONFIG.PCW_QSPI_GRP_IO1_ENABLE {0} \
CONFIG.PCW_QSPI_GRP_IO1_IO {<Select>} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {0} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_IO {<Select>} \
CONFIG.PCW_QSPI_GRP_SS1_ENABLE {0} \
CONFIG.PCW_QSPI_GRP_SS1_IO {<Select>} \
CONFIG.PCW_QSPI_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_QSPI_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_QSPI_QSPI_IO {<Select>} \
CONFIG.PCW_SD0_GRP_CD_ENABLE {0} \
CONFIG.PCW_SD0_GRP_CD_IO {<Select>} \
CONFIG.PCW_SD0_GRP_POW_ENABLE {0} \
CONFIG.PCW_SD0_GRP_POW_IO {<Select>} \
CONFIG.PCW_SD0_GRP_WP_ENABLE {0} \
CONFIG.PCW_SD0_GRP_WP_IO {<Select>} \
CONFIG.PCW_SD0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_SD0_SD0_IO {<Select>} \
CONFIG.PCW_SD1_GRP_CD_ENABLE {0} \
CONFIG.PCW_SD1_GRP_CD_IO {<Select>} \
CONFIG.PCW_SD1_GRP_POW_ENABLE {0} \
CONFIG.PCW_SD1_GRP_POW_IO {<Select>} \
CONFIG.PCW_SD1_GRP_WP_ENABLE {0} \
CONFIG.PCW_SD1_GRP_WP_IO {<Select>} \
CONFIG.PCW_SD1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_SD1_SD1_IO {<Select>} \
CONFIG.PCW_SDIO_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_SDIO_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_SMC_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_SMC_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_SMC_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_SPI0_GRP_SS0_ENABLE {0} \
CONFIG.PCW_SPI0_GRP_SS0_IO {<Select>} \
CONFIG.PCW_SPI0_GRP_SS1_ENABLE {0} \
CONFIG.PCW_SPI0_GRP_SS1_IO {<Select>} \
CONFIG.PCW_SPI0_GRP_SS2_ENABLE {0} \
CONFIG.PCW_SPI0_GRP_SS2_IO {<Select>} \
CONFIG.PCW_SPI0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_SPI0_SPI0_IO {<Select>} \
CONFIG.PCW_SPI1_GRP_SS0_ENABLE {0} \
CONFIG.PCW_SPI1_GRP_SS0_IO {<Select>} \
CONFIG.PCW_SPI1_GRP_SS1_ENABLE {0} \
CONFIG.PCW_SPI1_GRP_SS1_IO {<Select>} \
CONFIG.PCW_SPI1_GRP_SS2_ENABLE {0} \
CONFIG.PCW_SPI1_GRP_SS2_IO {<Select>} \
CONFIG.PCW_SPI1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_SPI1_SPI1_IO {<Select>} \
CONFIG.PCW_SPI_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_SPI_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_SPI_PERIPHERAL_FREQMHZ {166.666666} \
CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
CONFIG.PCW_S_AXI_HP1_DATA_WIDTH {64} \
CONFIG.PCW_S_AXI_HP2_DATA_WIDTH {64} \
CONFIG.PCW_S_AXI_HP3_DATA_WIDTH {64} \
CONFIG.PCW_TPIU_PERIPHERAL_CLKSRC {External} \
CONFIG.PCW_TPIU_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TPIU_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_TRACE_GRP_16BIT_ENABLE {0} \
CONFIG.PCW_TRACE_GRP_16BIT_IO {<Select>} \
CONFIG.PCW_TRACE_GRP_2BIT_ENABLE {0} \
CONFIG.PCW_TRACE_GRP_2BIT_IO {<Select>} \
CONFIG.PCW_TRACE_GRP_32BIT_ENABLE {0} \
CONFIG.PCW_TRACE_GRP_32BIT_IO {<Select>} \
CONFIG.PCW_TRACE_GRP_4BIT_ENABLE {0} \
CONFIG.PCW_TRACE_GRP_4BIT_IO {<Select>} \
CONFIG.PCW_TRACE_GRP_8BIT_ENABLE {0} \
CONFIG.PCW_TRACE_GRP_8BIT_IO {<Select>} \
CONFIG.PCW_TRACE_INTERNAL_WIDTH {2} \
CONFIG.PCW_TRACE_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_TRACE_TRACE_IO {<Select>} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_TTC0_TTC0_IO {<Select>} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_TTC1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_TTC1_TTC1_IO {<Select>} \
CONFIG.PCW_TTC_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_UART0_BAUD_RATE {115200} \
CONFIG.PCW_UART0_GRP_FULL_ENABLE {0} \
CONFIG.PCW_UART0_GRP_FULL_IO {<Select>} \
CONFIG.PCW_UART0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_UART0_UART0_IO {<Select>} \
CONFIG.PCW_UART1_BAUD_RATE {115200} \
CONFIG.PCW_UART1_GRP_FULL_ENABLE {0} \
CONFIG.PCW_UART1_GRP_FULL_IO {<Select>} \
CONFIG.PCW_UART1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_UART1_UART1_IO {<Select>} \
CONFIG.PCW_UART_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_UART_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_UART_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_UIPARAM_DDR_ADV_ENABLE {0} \
CONFIG.PCW_UIPARAM_DDR_AL {0} \
CONFIG.PCW_UIPARAM_DDR_BANK_ADDR_COUNT {3} \
CONFIG.PCW_UIPARAM_DDR_BL {8} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 {0.0} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 {0.0} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2 {0.0} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3 {0.0} \
CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit} \
CONFIG.PCW_UIPARAM_DDR_CL {7} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PACKAGE_LENGTH {61.0905} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PACKAGE_LENGTH {61.0905} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PACKAGE_LENGTH {61.0905} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PACKAGE_LENGTH {61.0905} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_STOP_EN {0} \
CONFIG.PCW_UIPARAM_DDR_COL_ADDR_COUNT {10} \
CONFIG.PCW_UIPARAM_DDR_CWL {6} \
CONFIG.PCW_UIPARAM_DDR_DEVICE_CAPACITY {1024 MBits} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_PACKAGE_LENGTH {68.4725} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_PACKAGE_LENGTH {71.086} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_PACKAGE_LENGTH {66.794} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_PACKAGE_LENGTH {108.7385} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 {0.0} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 {0.0} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 {0.0} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 {0.0} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_PACKAGE_LENGTH {64.1705} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_PACKAGE_LENGTH {63.686} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_PACKAGE_LENGTH {68.46} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_LENGTH_MM {0} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_PACKAGE_LENGTH {105.4895} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_PROPOGATION_DELAY {160} \
CONFIG.PCW_UIPARAM_DDR_DRAM_WIDTH {8 Bits} \
CONFIG.PCW_UIPARAM_DDR_ECC {Disabled} \
CONFIG.PCW_UIPARAM_DDR_ENABLE {1} \
CONFIG.PCW_UIPARAM_DDR_FREQ_MHZ {533.333333} \
CONFIG.PCW_UIPARAM_DDR_HIGH_TEMP {Normal (0-85)} \
CONFIG.PCW_UIPARAM_DDR_MEMORY_TYPE {DDR 3} \
CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41J128M8 JP-125} \
CONFIG.PCW_UIPARAM_DDR_ROW_ADDR_COUNT {14} \
CONFIG.PCW_UIPARAM_DDR_SPEED_BIN {DDR3_1066F} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL {1} \
CONFIG.PCW_UIPARAM_DDR_T_FAW {30.0} \
CONFIG.PCW_UIPARAM_DDR_T_RAS_MIN {35.0} \
CONFIG.PCW_UIPARAM_DDR_T_RC {48.75} \
CONFIG.PCW_UIPARAM_DDR_T_RCD {7} \
CONFIG.PCW_UIPARAM_DDR_T_RP {7} \
CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF {0} \
CONFIG.PCW_USB0_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_USB0_PERIPHERAL_FREQMHZ {60} \
CONFIG.PCW_USB0_RESET_ENABLE {0} \
CONFIG.PCW_USB0_RESET_IO {<Select>} \
CONFIG.PCW_USB0_USB0_IO {<Select>} \
CONFIG.PCW_USB1_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_USB1_PERIPHERAL_FREQMHZ {60} \
CONFIG.PCW_USB1_RESET_ENABLE {0} \
CONFIG.PCW_USB1_RESET_IO {<Select>} \
CONFIG.PCW_USB1_USB1_IO {<Select>} \
CONFIG.PCW_USB_RESET_ENABLE {1} \
CONFIG.PCW_USB_RESET_POLARITY {Active Low} \
CONFIG.PCW_USB_RESET_SELECT {<Select>} \
CONFIG.PCW_USE_CROSS_TRIGGER {0} \
CONFIG.PCW_USE_DEFAULT_ACP_USER_VAL {1} \
CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
CONFIG.PCW_USE_M_AXI_GP0 {1} \
CONFIG.PCW_USE_S_AXI_ACP {1} \
CONFIG.PCW_USE_S_AXI_GP0 {0} \
CONFIG.PCW_USE_S_AXI_HP0 {1} \
CONFIG.PCW_USE_S_AXI_HP1 {1} \
CONFIG.PCW_WDT_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_WDT_PERIPHERAL_DIVISOR0 {1} \
CONFIG.PCW_WDT_PERIPHERAL_ENABLE {0} \
CONFIG.PCW_WDT_PERIPHERAL_FREQMHZ {133.333333} \
CONFIG.PCW_WDT_WDT_IO {<Select>} \
 ] [get_bd_cells processing_system7_0]

  # Need to retain value_src of defaults
  set_property -dict [ list \
CONFIG.PCW_ACT_CAN_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_DCI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_ENET0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_ENET1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_FPGA3_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_PCAP_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_QSPI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_SDIO_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_SMC_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_SPI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_TPIU_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ACT_UART_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_APU_CLK_RATIO_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_APU_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ARMPLL_CTRL_FBDIV.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN0_CAN0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN0_GRP_CLK_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN0_GRP_CLK_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN0_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN1_CAN1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN1_GRP_CLK_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN1_GRP_CLK_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN1_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CAN_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CLK0_FREQ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CLK1_FREQ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CLK2_FREQ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CLK3_FREQ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CPU_CPU_6X4X_MAX_RANGE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CPU_CPU_PLL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CPU_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CPU_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DCI_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DCI_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DCI_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DCI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDRPLL_CTRL_FBDIV.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_DDR_PLL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_HPRLPR_QUEUE_PARTITION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_HPR_TO_CRITICAL_PRIORITY_LEVEL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_LPR_TO_CRITICAL_PRIORITY_LEVEL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PORT0_HPR_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PORT1_HPR_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PORT2_HPR_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PORT3_HPR_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_READPORT_0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_READPORT_1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_READPORT_2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_READPORT_3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_PRIORITY_WRITEPORT_3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_RAM_HIGHADDR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_DDR_WRITE_TO_CRITICAL_PRIORITY_LEVEL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_ENET0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_GRP_MDIO_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_GRP_MDIO_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET0_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_ENET1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_GRP_MDIO_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_GRP_MDIO_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET1_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET_RESET_POLARITY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_ENET_RESET_SELECT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_EN_4K_TIMER.VALUE_SRC {DEFAULT} \
CONFIG.PCW_EN_EMIO_GPIO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK0_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK1_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK1_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK1_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK2_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK2_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK2_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK3_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK3_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK3_PERIPHERAL_DIVISOR1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK_CLK0_BUF.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK_CLK1_BUF.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FCLK_CLK2_BUF.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FPGA3_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FPGA_FCLK0_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FPGA_FCLK1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_FPGA_FCLK2_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_GPIO_MIO_GPIO_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_GPIO_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_GRP_INT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_GRP_INT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_I2C0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C0_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_GRP_INT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_GRP_INT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_I2C1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C1_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C_RESET_POLARITY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_I2C_RESET_SELECT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_IOPLL_CTRL_FBDIV.VALUE_SRC {DEFAULT} \
CONFIG.PCW_IO_IO_PLL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_0_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_0_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_0_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_0_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_10_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_10_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_10_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_10_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_11_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_11_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_11_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_11_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_12_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_12_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_12_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_12_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_13_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_13_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_13_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_13_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_14_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_14_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_14_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_14_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_15_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_15_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_15_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_15_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_16_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_16_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_16_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_16_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_17_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_17_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_17_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_17_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_18_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_18_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_18_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_18_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_19_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_19_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_19_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_19_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_1_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_1_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_1_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_1_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_20_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_20_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_20_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_20_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_21_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_21_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_21_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_21_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_22_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_22_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_22_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_22_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_23_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_23_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_23_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_23_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_24_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_24_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_24_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_24_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_25_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_25_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_25_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_25_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_26_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_26_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_26_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_26_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_27_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_27_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_27_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_27_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_28_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_28_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_28_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_28_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_29_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_29_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_29_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_29_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_2_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_2_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_2_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_2_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_30_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_30_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_30_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_30_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_31_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_31_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_31_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_31_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_32_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_32_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_32_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_32_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_33_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_33_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_33_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_33_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_34_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_34_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_34_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_34_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_35_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_35_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_35_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_35_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_36_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_36_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_36_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_36_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_37_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_37_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_37_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_37_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_38_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_38_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_38_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_38_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_39_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_39_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_39_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_39_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_3_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_3_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_3_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_3_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_40_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_40_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_40_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_40_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_41_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_41_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_41_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_41_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_42_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_42_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_42_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_42_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_43_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_43_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_43_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_43_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_44_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_44_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_44_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_44_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_45_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_45_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_45_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_45_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_46_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_46_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_46_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_46_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_47_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_47_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_47_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_47_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_48_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_48_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_48_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_48_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_49_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_49_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_49_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_49_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_4_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_4_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_4_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_4_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_50_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_50_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_50_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_50_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_51_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_51_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_51_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_51_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_52_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_52_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_52_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_52_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_53_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_53_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_53_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_53_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_5_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_5_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_5_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_5_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_6_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_6_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_6_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_6_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_7_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_7_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_7_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_7_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_8_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_8_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_8_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_8_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_9_DIRECTION.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_9_IOTYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_9_PULLUP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_9_SLEW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_TREE_PERIPHERALS.VALUE_SRC {DEFAULT} \
CONFIG.PCW_MIO_TREE_SIGNALS.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_AR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_CLR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_REA.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_RR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_WC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_CYCLES_T_WP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_GRP_D8_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_GRP_D8_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_NAND_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NAND_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_CEOE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_PC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_TR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_WC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_T_WP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS0_WE_TIME.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_CEOE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_PC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_TR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_WC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_T_WP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_CS1_WE_TIME.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_A25_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_A25_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_CS0_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_CS0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_CS1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_CS1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_CS0_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_CS0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_CS1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_CS1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_INT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_GRP_SRAM_INT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_NOR_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_CEOE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_PC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_TR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_WC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_T_WP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS0_WE_TIME.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_CEOE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_PC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_TR.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_WC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_T_WP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_NOR_SRAM_CS1_WE_TIME.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_BOARD_DELAY3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PACKAGE_DDR_DQS_TO_CLK_DELAY_3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PCAP_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PCAP_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PCAP_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PERIPHERAL_BOARD_PRESET.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PJTAG_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PJTAG_PJTAG_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PLL_BYPASSMODE_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PRESET_BANK0_VOLTAGE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_PRESET_BANK1_VOLTAGE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_FBCLK_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_IO1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_IO1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_SS1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_GRP_SS1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_QSPI_QSPI_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_CD_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_CD_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_POW_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_POW_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_WP_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_GRP_WP_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD0_SD0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_CD_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_CD_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_POW_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_POW_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_WP_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_GRP_WP_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SD1_SD1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SDIO_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SDIO_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SMC_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SMC_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SMC_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS0_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS2_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_GRP_SS2_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI0_SPI0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS0_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS1_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS2_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_GRP_SS2_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI1_SPI1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_SPI_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_S_AXI_HP0_DATA_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_S_AXI_HP1_DATA_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_S_AXI_HP2_DATA_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_S_AXI_HP3_DATA_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TPIU_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TPIU_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TPIU_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_16BIT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_16BIT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_2BIT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_2BIT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_32BIT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_32BIT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_4BIT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_4BIT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_8BIT_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_GRP_8BIT_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_INTERNAL_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TRACE_TRACE_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC0_TTC0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_CLK2_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC1_TTC1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_TTC_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART0_BAUD_RATE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART0_GRP_FULL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART0_GRP_FULL_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART0_UART0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART1_BAUD_RATE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART1_GRP_FULL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART1_GRP_FULL_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART1_UART1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UART_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_ADV_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_AL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BANK_ADDR_COUNT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_STOP_EN.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_COL_ADDR_COUNT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_CWL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DEVICE_CAPACITY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_LENGTH_MM.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_PACKAGE_LENGTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_PROPOGATION_DELAY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_DRAM_WIDTH.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_ECC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_FREQ_MHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_HIGH_TEMP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_MEMORY_TYPE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_PARTNO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_ROW_ADDR_COUNT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_SPEED_BIN.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_T_FAW.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_T_RAS_MIN.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_T_RC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_T_RCD.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_T_RP.VALUE_SRC {DEFAULT} \
CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB0_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB0_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB0_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB0_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB0_USB0_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB1_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB1_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB1_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB1_RESET_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB1_USB1_IO.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB_RESET_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB_RESET_POLARITY.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USB_RESET_SELECT.VALUE_SRC {DEFAULT} \
CONFIG.PCW_USE_CROSS_TRIGGER.VALUE_SRC {DEFAULT} \
CONFIG.PCW_WDT_PERIPHERAL_CLKSRC.VALUE_SRC {DEFAULT} \
CONFIG.PCW_WDT_PERIPHERAL_DIVISOR0.VALUE_SRC {DEFAULT} \
CONFIG.PCW_WDT_PERIPHERAL_ENABLE.VALUE_SRC {DEFAULT} \
CONFIG.PCW_WDT_PERIPHERAL_FREQMHZ.VALUE_SRC {DEFAULT} \
CONFIG.PCW_WDT_WDT_IO.VALUE_SRC {DEFAULT} \
 ] [get_bd_cells processing_system7_0]

connect_bd_intf_net -intf_net S00_AXI_1 [get_bd_intf_pins axi_dmac_0/m_src_axi] [get_bd_intf_pins axi_interconnect_0/S00_AXI]
  connect_bd_intf_net -intf_net axi_dmac_0_m_axis [get_bd_intf_pins axi_dmac_0/m_axis] [get_bd_intf_pins axi_hdmi_tx_0/s_axis]
  connect_bd_intf_net -intf_net axi_dmac_1_m_dest_axi [get_bd_intf_pins axi_dmac_rx/m_dest_axi] [get_bd_intf_pins axi_interconnect_1/S00_AXI]
  connect_bd_intf_net -intf_net axi_gpio_0_GPIO2 [get_bd_intf_ports GPIO_BTN] [get_bd_intf_pins axi_gpio_0/GPIO2]
 

  connect_bd_intf_net -intf_net axi_hwt_M07_AXI [get_bd_intf_pins axi_dmac_0/s_axi] [get_bd_intf_pins axi_hwt/M07_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M08_AXI [get_bd_intf_pins axi_hdmi_tx_0/s_axi] [get_bd_intf_pins axi_hwt/M08_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M09_AXI [get_bd_intf_pins axi_clkgen_0/s_axi] [get_bd_intf_pins axi_hwt/M09_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M10_AXI [get_bd_intf_pins axi_hwt/M10_AXI] [get_bd_intf_pins iic_main/S_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M11_AXI [get_bd_intf_pins axi_hwt/M11_AXI] [get_bd_intf_pins iic_fmc/S_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M12_AXI [get_bd_intf_pins axi_hdmi_rx_0/s_axi] [get_bd_intf_pins axi_hwt/M12_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M13_AXI [get_bd_intf_pins axi_dmac_rx/s_axi] [get_bd_intf_pins axi_hwt/M13_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M14_AXI [get_bd_intf_pins axi_hwt/M14_AXI] [get_bd_intf_pins axi_iic_ad7611/S_AXI]
  connect_bd_intf_net -intf_net axi_hwt_M15_AXI [get_bd_intf_pins axi_gpio_0/S_AXI] [get_bd_intf_pins axi_hwt/M15_AXI]
  
  connect_bd_intf_net -intf_net axi_iic_0_IIC [get_bd_intf_ports IIC] [get_bd_intf_pins iic_main/IIC]
  connect_bd_intf_net -intf_net axi_iic_0_IIC1 [get_bd_intf_ports IIC_2] [get_bd_intf_pins axi_iic_ad7611/IIC]
  connect_bd_intf_net -intf_net axi_iic_1_IIC [get_bd_intf_ports IIC_1] [get_bd_intf_pins iic_fmc/IIC]
  connect_bd_intf_net -intf_net axi_interconnect_0_M00_AXI [get_bd_intf_pins axi_interconnect_0/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP0]
  connect_bd_intf_net -intf_net axi_interconnect_1_M00_AXI [get_bd_intf_pins axi_interconnect_1/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP1]
  
  connect_bd_net -net SW0_1 [get_bd_ports SW0] [get_bd_pins xlconcat_1/In1]
  connect_bd_net -net axi_clkgen_0_clk_0 [get_bd_pins axi_clkgen_0/clk_0] [get_bd_pins axi_hdmi_tx_0/hdmi_clk]
  connect_bd_net -net axi_dmac_0_irq [get_bd_pins axi_dmac_0/irq] [get_bd_pins xlconcat_0/In13]
  connect_bd_net -net axi_dmac_1_fifo_wr_overflow [get_bd_ports LD5] [get_bd_pins axi_dmac_rx/fifo_wr_overflow] [get_bd_pins axi_hdmi_rx_0/hdmi_dma_ovf]
  connect_bd_net -net axi_dmac_1_irq [get_bd_pins axi_dmac_rx/irq] [get_bd_pins xlconcat_0/In10]
  connect_bd_net -net axi_gpio_0_ip2intc_irpt [get_bd_pins axi_gpio_0/ip2intc_irpt] [get_bd_pins xlconcat_0/In8]
  connect_bd_net -net axi_hdmi_rx_0_hdmi_clk [get_bd_pins axi_dmac_rx/fifo_wr_clk] [get_bd_pins axi_hdmi_rx_0/hdmi_clk]
  connect_bd_net -net axi_hdmi_rx_0_hdmi_dma_data [get_bd_pins axi_dmac_rx/fifo_wr_din] [get_bd_pins axi_hdmi_rx_0/hdmi_dma_data]
  connect_bd_net -net axi_hdmi_rx_0_hdmi_dma_de [get_bd_ports LD2] [get_bd_pins axi_dmac_rx/fifo_wr_en] [get_bd_pins axi_hdmi_rx_0/hdmi_dma_de]
  connect_bd_net -net axi_hdmi_rx_0_hdmi_dma_sof [get_bd_ports LD1] [get_bd_pins axi_dmac_rx/fifo_wr_sync] [get_bd_pins axi_hdmi_rx_0/hdmi_dma_sof]
  connect_bd_net -net axi_hdmi_tx_0_hdmi_16_data [get_bd_ports hdmi_16_data] [get_bd_pins axi_hdmi_tx_0/hdmi_16_data]
  connect_bd_net -net axi_hdmi_tx_0_hdmi_16_data_e [get_bd_ports hdmi_16_data_e] [get_bd_pins axi_hdmi_tx_0/hdmi_16_data_e]
  connect_bd_net -net axi_hdmi_tx_0_hdmi_16_hsync [get_bd_ports hdmi_16_hsync] [get_bd_pins axi_hdmi_tx_0/hdmi_16_hsync]
  connect_bd_net -net axi_hdmi_tx_0_hdmi_16_vsync [get_bd_ports hdmi_16_vsync] [get_bd_pins axi_hdmi_tx_0/hdmi_16_vsync]
  connect_bd_net -net axi_hdmi_tx_0_hdmi_out_clk [get_bd_ports hdmi_out_clk] [get_bd_pins axi_hdmi_tx_0/hdmi_out_clk]
  connect_bd_net -net axi_iic_0_iic2intc_irpt [get_bd_pins iic_main/iic2intc_irpt] [get_bd_pins xlconcat_0/In11]
  connect_bd_net -net axi_iic_0_iic2intc_irpt1 [get_bd_pins axi_iic_ad7611/iic2intc_irpt] [get_bd_pins xlconcat_0/In9]
  connect_bd_net -net axi_iic_1_iic2intc_irpt [get_bd_pins iic_fmc/iic2intc_irpt] [get_bd_pins xlconcat_0/In12]
  connect_bd_net -net gnd_1_dout [get_bd_pins gnd_1/dout] [get_bd_pins xlconcat_1/In2] [get_bd_pins xlconcat_1/In3] [get_bd_pins xlconcat_1/In4] [get_bd_pins xlconcat_1/In5] [get_bd_pins xlconcat_1/In6] [get_bd_pins xlconcat_1/In7] [get_bd_pins xlconcat_1/In8] [get_bd_pins xlconcat_1/In9] [get_bd_pins xlconcat_1/In10]
  connect_bd_net -net hdmi_int_1 [get_bd_ports LD3] [get_bd_ports hdmi_int] [get_bd_pins xlconcat_1/In0]
  connect_bd_net -net hdmi_rx_clk_1 [get_bd_ports hdmi_rx_clk] [get_bd_pins axi_hdmi_rx_0/hdmi_rx_clk]
  connect_bd_net -net hdmi_rx_data_1 [get_bd_ports hdmi_rx_data] [get_bd_pins axi_hdmi_rx_0/hdmi_rx_data] [get_bd_pins xlslice_0/Din]
 
  connect_bd_net -net processing_system7_0_FCLK_CLK1 [get_bd_pins axi_clkgen_0/clk] [get_bd_pins processing_system7_0/FCLK_CLK1]
  connect_bd_net -net processing_system7_0_FCLK_CLK2 [get_bd_pins axi_dmac_rx/m_dest_axi_aclk] [get_bd_pins axi_interconnect_1/ACLK] [get_bd_pins axi_interconnect_1/M00_ACLK] [get_bd_pins axi_interconnect_1/S00_ACLK] [get_bd_pins processing_system7_0/FCLK_CLK2] [get_bd_pins processing_system7_0/S_AXI_HP1_ACLK]
  connect_bd_net -net xlconcat_0_dout [get_bd_pins processing_system7_0/IRQ_F2P] [get_bd_pins xlconcat_0/dout]
  connect_bd_net -net xlconcat_1_dout [get_bd_pins processing_system7_0/GPIO_I] [get_bd_pins xlconcat_1/dout]
  connect_bd_net -net xlconstant_0_dout [get_bd_pins xlconcat_0/In0] [get_bd_pins xlconcat_0/In1] [get_bd_pins xlconcat_0/In2] [get_bd_pins xlconcat_0/In3] [get_bd_pins xlconcat_0/In4] [get_bd_pins xlconcat_0/In5] [get_bd_pins xlconcat_0/In6] [get_bd_pins xlconcat_0/In7] [get_bd_pins xlconstant_0/dout]
  connect_bd_net -net xlconstant_1_dout [get_bd_ports LD0] [get_bd_pins xlconstant_1/dout]
  connect_bd_net -net xlconstant_2_dout [get_bd_pins axi_hdmi_rx_0/hdmi_dma_unf] [get_bd_pins gnd/dout]
  connect_bd_net -net xlslice_0_Dout [get_bd_ports LD4] [get_bd_pins xlslice_0/Dout]

#Set Address Ranges
  create_bd_addr_seg -range 0x20000000 -offset 0x00000000 [get_bd_addr_spaces axi_dmac_0/m_src_axi] [get_bd_addr_segs processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM] SEG_processing_system7_0_HP0_DDR_LOWOCM
  create_bd_addr_seg -range 0x20000000 -offset 0x00000000 [get_bd_addr_spaces axi_dmac_rx/m_dest_axi] [get_bd_addr_segs processing_system7_0/S_AXI_HP1/HP1_DDR_LOWOCM] SEG_processing_system7_0_HP1_DDR_LOWOCM
  create_bd_addr_seg -range 0x00010000 -offset 0x79000000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_clkgen_0/s_axi/axi_lite] SEG_axi_clkgen_0_axi_lite
  create_bd_addr_seg -range 0x00001000 -offset 0x43000000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_dmac_0/s_axi/axi_lite] SEG_axi_dmac_0_axi_lite
  create_bd_addr_seg -range 0x00001000 -offset 0x43C20000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_dmac_rx/s_axi/axi_lite] SEG_axi_dmac_rx_axi_lite
  create_bd_addr_seg -range 0x00001000 -offset 0x41200000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_gpio_0/S_AXI/Reg] SEG_axi_gpio_0_Reg
  create_bd_addr_seg -range 0x00010000 -offset 0x75C20000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_hdmi_rx_0/s_axi/axi_lite] SEG_axi_hdmi_rx_0_axi_lite
  create_bd_addr_seg -range 0x00010000 -offset 0x70E00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_hdmi_tx_0/s_axi/axi_lite] SEG_axi_hdmi_tx_0_axi_lite
  create_bd_addr_seg -range 0x00001000 -offset 0x43C40000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_iic_ad7611/S_AXI/Reg] SEG_axi_iic_ad7611_Reg
  create_bd_addr_seg -range 0x00001000 -offset 0x41620000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs iic_fmc/S_AXI/Reg] SEG_iic_fmc_Reg
  create_bd_addr_seg -range 0x00001000 -offset 0x41600000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs iic_main/S_AXI/Reg] SEG_iic_main_Reg
  create_bd_addr_seg -range 0x20000000 -offset 0x00000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M_AXI] [get_bd_addr_segs processing_system7_0/S_AXI_ACP/ACP_DDR_LOWOCM] SEG_processing_system7_0_ACP_DDR_LOWOCM
  create_bd_addr_seg -range 0x40000000 -offset 0x40000000 [get_bd_addr_spaces reconos_memif_memory_controller_0/M_AXI] [get_bd_addr_segs processing_system7_0/S_AXI_ACP/ACP_M_AXI_GP0] SEG_processing_system7_0_ACP_M_AXI_GP0


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
    connect_bd_intf_net -intf_net axi_hwt_M05_AXI [get_bd_intf_pins axi_hwt/M05_AXI] [get_bd_intf_pins servo_0/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M06_AXI [get_bd_intf_pins axi_hwt/M06_AXI] [get_bd_intf_pins touch_0/S00_AXI]

    connect_bd_intf_net -intf_net axi_hwt_M16_AXI [get_bd_intf_pins axi_hwt/M16_AXI] [get_bd_intf_pins servo_1/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M17_AXI [get_bd_intf_pins axi_hwt/M17_AXI] [get_bd_intf_pins touch_1/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M18_AXI [get_bd_intf_pins axi_hwt/M18_AXI] [get_bd_intf_pins servo_2/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M19_AXI [get_bd_intf_pins axi_hwt/M19_AXI] [get_bd_intf_pins touch_2/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M19_AXI [get_bd_intf_pins axi_hwt/M19_AXI] [get_bd_intf_pins touch_2/S00_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M20_AXI [get_bd_intf_pins axi_hwt/M20_AXI] [get_bd_intf_pins axi_timer_0/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M21_AXI [get_bd_intf_pins axi_hwt/M21_AXI] [get_bd_intf_pins axi_timer_1/S_AXI]
    connect_bd_intf_net -intf_net axi_hwt_M22_AXI [get_bd_intf_pins axi_hwt/M22_AXI] [get_bd_intf_pins difference_measurement_timer_0/S00_AXI]



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
                            [get_bd_pins axi_hwt/M07_ACLK] \
                            [get_bd_pins axi_hwt/M08_ACLK] \
                            [get_bd_pins axi_hwt/M09_ACLK] \
                            [get_bd_pins axi_hwt/M10_ACLK] \
                            [get_bd_pins axi_hwt/M11_ACLK] \
                            [get_bd_pins axi_hwt/M12_ACLK] \
                            [get_bd_pins axi_hwt/M13_ACLK] \
                            [get_bd_pins axi_hwt/M14_ACLK] \
                            [get_bd_pins axi_hwt/M15_ACLK] \
                            [get_bd_pins axi_hwt/M16_ACLK] \
                            [get_bd_pins axi_hwt/M17_ACLK] \
                            [get_bd_pins axi_hwt/M18_ACLK] \
                            [get_bd_pins axi_hwt/M19_ACLK] \
                            [get_bd_pins axi_hwt/M20_ACLK] \
                            [get_bd_pins axi_hwt/M21_ACLK] \
                            [get_bd_pins axi_hwt/M22_ACLK] \
                            [get_bd_pins axi_hwt/S00_ACLK] \
                            [get_bd_pins axi_dmac_0/s_axi_aclk] \
                            [get_bd_pins axi_dmac_0/m_src_axi_aclk] \
                            [get_bd_pins axi_dmac_0/m_axis_aclk] \
                            [get_bd_pins axi_dmac_rx/s_axi_aclk] \
                            [get_bd_pins axi_gpio_0/s_axi_aclk] \
                            [get_bd_pins axi_hdmi_rx_0/s_axi_aclk] \
                            [get_bd_pins axi_hdmi_tx_0/s_axi_aclk] \
                            [get_bd_pins axi_hdmi_tx_0/vdma_clk] \
                            [get_bd_pins axi_iic_ad7611/s_axi_aclk] \
                            [get_bd_pins iic_fmc/s_axi_aclk] \
                            [get_bd_pins iic_main/s_axi_aclk] \
                            [get_bd_pins /processing_system7_0/S_AXI_HP0_ACLK] \
                            [get_bd_pins reconos_osif_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_osif_intc_0/S_AXI_ACLK] \
                            [get_bd_pins reconos_proc_control_0/S_AXI_ACLK] \
                            [get_bd_pins reset_0/slowest_sync_clk] \
                            [get_bd_pins timer_0/S_AXI_ACLK] \
                            [get_bd_pins servo_0/s00_axi_aclk] \
                            [get_bd_pins touch_0/s00_axi_aclk] \
                            [get_bd_pins servo_1/s00_axi_aclk] \
                            [get_bd_pins touch_1/s00_axi_aclk] \
                            [get_bd_pins servo_2/s00_axi_aclk] \
                            [get_bd_pins touch_2/s00_axi_aclk] \
                            [get_bd_pins axi_timer_0/s_axi_aclk] \
                            [get_bd_pins axi_timer_1/s_axi_aclk] \
                            [get_bd_pins difference_measurement_timer_0/s00_axi_aclk] \
                            [get_bd_pins axi_clkgen_0/s_axi_aclk]\
                            [get_bd_pins axi_interconnect_0/ACLK] \
                            [get_bd_pins axi_interconnect_0/S00_ACLK] \
                            [get_bd_pins axi_interconnect_0/M00_ACLK] 
                            
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
                            [get_bd_pins axi_hwt/M07_ARESETN] \
                            [get_bd_pins axi_hwt/M08_ARESETN] \
                            [get_bd_pins axi_hwt/M09_ARESETN] \
                            [get_bd_pins axi_hwt/M10_ARESETN] \
                            [get_bd_pins axi_hwt/M11_ARESETN] \
                            [get_bd_pins axi_hwt/M12_ARESETN] \
                            [get_bd_pins axi_hwt/M13_ARESETN] \
                            [get_bd_pins axi_hwt/M14_ARESETN] \
                            [get_bd_pins axi_hwt/M15_ARESETN] \
                            [get_bd_pins axi_hwt/M16_ARESETN] \
                            [get_bd_pins axi_hwt/M17_ARESETN] \
                            [get_bd_pins axi_hwt/M18_ARESETN] \
                            [get_bd_pins axi_hwt/M19_ARESETN] \
                            [get_bd_pins axi_hwt/M20_ARESETN] \
                            [get_bd_pins axi_hwt/M21_ARESETN] \
                            [get_bd_pins axi_hwt/M22_ARESETN] \
                            [get_bd_pins axi_hwt/S00_ARESETN] \
                            [get_bd_pins axi_interconnect_0/M00_ARESETN] \
                            [get_bd_pins axi_interconnect_0/S00_ARESETN] \
                            [get_bd_pins axi_interconnect_0/ARESETN] \
                            [get_bd_pins axi_interconnect_1/M00_ARESETN] \
                            [get_bd_pins axi_interconnect_1/S00_ARESETN] \
                            [get_bd_pins axi_interconnect_1/ARESETN] \
                            [get_bd_pins axi_hdmi_tx_0/s_axi_aresetn] \
                            [get_bd_pins axi_hdmi_rx_0/s_axi_aresetn] \
                            [get_bd_pins axi_clkgen_0/s_axi_aresetn] \
                            [get_bd_pins axi_dmac_rx/s_axi_aresetn] \
                            [get_bd_pins axi_dmac_0/s_axi_aresetn] \
                            [get_bd_pins axi_timer_0/s_axi_aresetn] \
                            [get_bd_pins axi_timer_1/s_axi_aresetn] \
                            [get_bd_pins difference_measurement_timer_0/s00_axi_aresetn] 
                            
    # Proc_control resets
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst] [get_bd_pins reconos_memif_arbiter_0/SYS_Rst]
    connect_bd_net [get_bd_pins reconos_memif_mmu_zynq_0/SYS_Rst] [get_bd_pins reconos_proc_control_0/PROC_Sys_Rst]

    # ReconoOS Peripherals reset by peripheral_aresetn
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_clock_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins timer_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins servo_0/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins touch_0/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins servo_1/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins touch_1/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins servo_2/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins touch_2/s00_axi_aresetn]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_proc_control_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_intc_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_osif_0/S_AXI_ARESETN]
    connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins reconos_memif_memory_controller_0/M_AXI_ARESETN]

  
   
   
    connect_bd_net [get_bd_pins reconos_osif_intc_0/OSIF_INTC_Out] [get_bd_pins xlconcat_0/In14]
    connect_bd_net [get_bd_pins reconos_proc_control_0/PROC_Pgf_Int] [get_bd_pins xlconcat_0/In15]


    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_1
    set_property -dict [list CONFIG.DIN_TO {2} CONFIG.DIN_FROM {2} CONFIG.DIN_FROM {2} CONFIG.DOUT_WIDTH {1}] [get_bd_cells xlslice_1]
    
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_2   
    set_property -dict [list CONFIG.DIN_TO {2} CONFIG.DIN_FROM {2} CONFIG.DIN_FROM {2} CONFIG.DOUT_WIDTH {1}] [get_bd_cells xlslice_2]
    connect_bd_net [get_bd_pins xlslice_2/Dout] [get_bd_pins difference_measurement_timer_0/Capture_2]

    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 xlslice_3
    connect_bd_net [get_bd_pins xlslice_3/Dout] [get_bd_pins difference_measurement_timer_0/Capture_3]
    set_property -dict [list CONFIG.DIN_TO {3} CONFIG.DIN_FROM {3} CONFIG.DIN_FROM {3} CONFIG.DOUT_WIDTH {1}] [get_bd_cells xlslice_3]
    
    #connect_bd_net [get_bd_pins xlslice_2/Din] [get_bd_pins slot_12/debug_port]
    #connect_bd_net [get_bd_pins xlslice_3/Din] [get_bd_pins slot_12/debug_port]

    #
    # Memory Map of peripheperals
    #

    set_property -dict [list CONFIG.C_BASEADDR {0x6fe00000} CONFIG.C_HIGHADDR {0x6fe0ffff}] [get_bd_cells reconos_proc_control_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x75a00000} CONFIG.C_HIGHADDR {0x75a0ffff}] [get_bd_cells reconos_osif_0]   
    set_property -dict [list CONFIG.C_BASEADDR {0x7b400000} CONFIG.C_HIGHADDR {0x7b40ffff}] [get_bd_cells reconos_osif_intc_0]
    set_property -dict [list CONFIG.C_BASEADDR {0x69e00000} CONFIG.C_HIGHADDR {0x69e0ffff}] [get_bd_cells reconos_clock_0]
    
    create_bd_addr_seg -range 64K -offset 0x6FE00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_proc_control_0/S_AXI/reg0 }] SEG1
    create_bd_addr_seg -range 64K -offset 0x75a00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_osif_0/S_AXI/reg0 }] SEG2
    create_bd_addr_seg -range 64K -offset 0x64a00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {timer_0/S_AXI/reg0 }] SEG3
    create_bd_addr_seg -range 64K -offset 0x7b400000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_osif_intc_0/S_AXI/reg0 }] SEG4
    create_bd_addr_seg -range 64K -offset 0x69e00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {reconos_clock_0/S_AXI/reg0 }] SEG5
    create_bd_addr_seg -range 64K -offset 0x43C00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {servo_0/S00_AXI/S00_AXI_reg }] SEG6
    create_bd_addr_seg -range 64K -offset 0x43C10000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {touch_0/S00_AXI/S00_AXI_reg }] SEG7
    create_bd_addr_seg -range 64K -offset 0x43C70000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {servo_1/S00_AXI/S00_AXI_reg }] SEG9
    create_bd_addr_seg -range 64K -offset 0x43C30000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {touch_1/S00_AXI/S00_AXI_reg }] SEG10
    create_bd_addr_seg -range 64K -offset 0x43C60000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {servo_2/S00_AXI/S00_AXI_reg }] SEG11
    create_bd_addr_seg -range 64K -offset 0x43C50000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {touch_2/S00_AXI/S00_AXI_reg }] SEG12
    create_bd_addr_seg -range 64K -offset 0x42800000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {axi_timer_0/S_AXI/Reg }] SEG13
    create_bd_addr_seg -range 64K -offset 0x42810000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {axi_timer_1/S_AXI/Reg }] SEG14
    create_bd_addr_seg -range 64K -offset 0x43C80000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs {difference_measurement_timer_0/S00_AXI/S00_AXI_reg }] SEG15

    assign_bd_address [get_bd_addr_segs {processing_system7_0/S_AXI_ACP/ACP_DDR_LOWOCM }]
    assign_bd_address [get_bd_addr_segs {processing_system7_0/S_AXI_ACP/ACP_M_AXI_GP0 }]

    

   # connect_bd_net [get_bd_pins axi_timer_1/generateout0] \
  #                          [get_bd_pins slot_3/HWT_Trig] \
  #                          [get_bd_pins slot_4/HWT_Trig] \
  #                          [get_bd_pins slot_5/HWT_Trig] 

                            
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


