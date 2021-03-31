#                                                        ____  _____
#                            ________  _________  ____  / __ \/ ___/
#                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
#                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
#                         /_/   \___/\___/\____/_/ /_/\____//____/
# 
# ======================================================================
#
#   title:        ReconOS ip library script for Vivado
#
#   project:      ReconOS
#   author:       Sebastian Meisner, University of Paderborn
#   description:  This TCL script sets imports all ReconOS modules and 
#                 the hardware threads to an IP integrator library. The
#                 export.tcl then uses this library to create a working
#                 ReconOS system design.
#
# ======================================================================

<<reconos_preproc>>

proc create_fifo_interfaces {repo_path} {
	
	#
	# FIFO_M
	#
	ipx::create_abstraction_definition cs.upb.de reconos FIFO_M_rtl 1.0
	ipx::create_bus_definition cs.upb.de reconos FIFO_M 1.0
	set_property xml_file_name $repo_path/FIFO_M_rtl.xml [ipx::current_busabs]
	set_property xml_file_name $repo_path/FIFO_M.xml [ipx::current_busdef]
	set_property bus_type_vlnv cs.upb.de:reconos:FIFO_M:1.0 [ipx::current_busabs]
	set_property description {Master writes data, Slave accepts data} [ipx::current_busdef]
	
	#
	# FIFO_M_Data
	#
	ipx::add_bus_abstraction_port FIFO_M_Data [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property master_width 32 [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property slave_direction in [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property slave_width 32 [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property is_data true [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	set_property description {FIFO Master Data Signal} [ipx::get_bus_abstraction_ports FIFO_M_Data -of_objects [ipx::current_busabs]]
	
	#
	# FIFO_M_Full
	#
	ipx::add_bus_abstraction_port FIFO_M_Full [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property master_direction in [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property master_width 1 [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property slave_width 1 [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	set_property description {FIFO Master Full Signal} [ipx::get_bus_abstraction_ports FIFO_M_Full -of_objects [ipx::current_busabs]]
	
	#
	# FIFO_M_WE
	#
	ipx::add_bus_abstraction_port FIFO_M_WE [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property master_width 1 [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property slave_direction in [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property slave_width 1 [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	set_property description {FIFO Master Write Enable Signal} [ipx::get_bus_abstraction_ports FIFO_M_WE -of_objects [ipx::current_busabs]]
	
	ipx::save_abstraction_definition [ipx::current_busabs]
	ipx::save_bus_definition [ipx::current_busdef]
	
	#
	# FIFO_S
	#
	ipx::create_abstraction_definition cs.upb.de reconos FIFO_S_rtl 1.0
	ipx::create_bus_definition cs.upb.de reconos FIFO_S 1.0
	set_property xml_file_name $repo_path/FIFO_S_rtl.xml [ipx::current_busabs]
	set_property xml_file_name $repo_path/FIFO_S.xml [ipx::current_busdef]
	set_property bus_type_vlnv cs.upb.de:reconos:FIFO_S:1.0 [ipx::current_busabs]
	set_property description {Master reads data, Slave provides data} [ipx::current_busdef]
	
	#
	# FIFO_S_Data
	#
	ipx::add_bus_abstraction_port FIFO_S_Data [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property master_direction in [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property master_width 32 [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property slave_width 32 [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property is_data true [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	set_property description {FIFO Slave Data Signal} [ipx::get_bus_abstraction_ports FIFO_S_Data -of_objects [ipx::current_busabs]]
	
	#
	# FIFO_S_Empty
	#
	ipx::add_bus_abstraction_port FIFO_S_Empty [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property master_direction in [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property master_width 1 [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property slave_width 1 [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	set_property description {FIFO Slave Empty Signal} [ipx::get_bus_abstraction_ports FIFO_S_Empty -of_objects [ipx::current_busabs]]
	
	#
	# FIFO_S_RE
	#
	ipx::add_bus_abstraction_port FIFO_S_RE [ipx::current_busabs]
	set_property default_value 0 [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property master_presence required [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property master_width 1 [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property slave_presence required [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property slave_direction in [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property slave_width 1 [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	set_property description {FIFO Slave Read Enable Signal} [ipx::get_bus_abstraction_ports FIFO_S_RE -of_objects [ipx::current_busabs]]
	
	ipx::save_abstraction_definition [ipx::current_busabs]
	ipx::save_bus_definition [ipx::current_busdef]
	
}
proc load_fifo_interfaces {repo_path} {
	#
	# Opens interface definitions
	#
	ipx::open_ipxact_file $repo_path/FIFO_M.xml
	ipx::open_ipxact_file $repo_path/FIFO_S.xml
}


proc add_fifo_m_interface {path_to_ip bus_name full_signal_name data_signal_name we_signal_name mode} {
	
	ipx::current_core $path_to_ip/component.xml
	
	#
	# Bus MEMIF_Mem2Hwt_0  (FIFO_M type interface)
	#
	ipx::add_bus_interface $bus_name [ipx::current_core]
	set_property abstraction_type_vlnv cs.upb.de:reconos:FIFO_M_rtl:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property bus_type_vlnv cs.upb.de:reconos:FIFO_M:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	if { [string equal "master" $mode] == 1 } {
		set_property interface_mode master [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	}
	
	#
	# FIFO_M_WE
	#
	ipx::add_port_map FIFO_M_WE [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $we_signal_name [ipx::get_port_maps FIFO_M_WE -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# FIFO_M_Data
	#
	ipx::add_port_map FIFO_M_Data [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $data_signal_name [ipx::get_port_maps FIFO_M_Data -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# FIFO_M_Full
	#
	ipx::add_port_map FIFO_M_Full [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $full_signal_name [ipx::get_port_maps FIFO_M_Full -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# Save everything
	#
	ipx::create_xgui_files [ipx::current_core]
	ipx::update_checksums [ipx::current_core]
	ipx::save_core [ipx::current_core]
	
}

proc add_fifo_s_interface {path_to_ip bus_name empty_signal_name data_signal_name re_signal_name mode} {
	
	ipx::current_core $path_to_ip/component.xml
	
	#
	# Bus MEMIF_Hwt2Mem_0  (FIFO_S type interface)
	#
	ipx::add_bus_interface $bus_name [ipx::current_core]
	set_property abstraction_type_vlnv cs.upb.de:reconos:FIFO_S_rtl:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property bus_type_vlnv cs.upb.de:reconos:FIFO_S:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	if { [string equal "master" $mode] == 1 } {
		set_property interface_mode master [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	}
	
	#
	# FIFO_S_Empty
	#
	ipx::add_port_map FIFO_S_Empty [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $empty_signal_name [ipx::get_port_maps FIFO_S_Empty -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# FIFO_S_Data
	#
	ipx::add_port_map FIFO_S_Data [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $data_signal_name [ipx::get_port_maps FIFO_S_Data -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# FIFO_S_RE
	#
	ipx::add_port_map FIFO_S_RE [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
	set_property physical_name $re_signal_name [ipx::get_port_maps FIFO_S_RE -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]
	
	#
	# Save everything
	#
	ipx::create_xgui_files [ipx::current_core]
	ipx::update_checksums [ipx::current_core]
	ipx::save_core [ipx::current_core]
}

proc add_clock_interface { path_to_ip bus_name clock_signal_name } {
        
        ipx::current_core $path_to_ip/component.xml
        
        #
        # Clock interface instantiation
        #
        ipx::add_bus_interface $bus_name [ipx::current_core]
        set_property abstraction_type_vlnv xilinx.com:signal:clock_rtl:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
        set_property bus_type_vlnv xilinx.com:signal:clock:1.0 [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
        set_property interface_mode master [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
        
        #
        # Add Clock Port
        #
        ipx::add_port_map CLK [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]
        set_property physical_name $clock_signal_name [ipx::get_port_maps CLK -of_objects [ipx::get_bus_interfaces $bus_name -of_objects [ipx::current_core]]]

        
        #
	# Save everything
	#
	ipx::create_xgui_files [ipx::current_core]
	ipx::update_checksums [ipx::current_core]
	ipx::save_core [ipx::current_core]
}

proc import_pcore { repo_path ip_name {libs ""} } {
	set reconos_pcore $ip_name  ;#[file tail $argv]
	set reconos_pcore_name [string range $reconos_pcore 0 [expr [string length $reconos_pcore]-9] ] ;# cuts of version string
	set reconos_pcore_dir $repo_path ;#[file dirname $argv] 
	set temp_dir "/tmp/reconos_tmp/"

	puts "\[RDK\] $reconos_pcore $reconos_pcore_name $reconos_pcore_dir $temp_dir"
	
	if { $reconos_pcore_name == "reconos" } {
		ipx::infer_core -set_current true -as_library true -vendor cs.upb.de -taxonomy /ReconOS  $reconos_pcore_dir/$reconos_pcore
		set_property display_name ReconosLib [ipx::current_core]
	} else {
		ipx::infer_core -set_current true -as_library false -vendor cs.upb.de -taxonomy /ReconOS  $reconos_pcore_dir/$reconos_pcore
	}

	puts "\[RDK\] After infer_core"
	#set_property vendor 				cs.upb.de 				[ipx::current_core]
	set_property library 				reconos 				[ipx::current_core]
	set_property company_url 			http://www.reconos.de 	[ipx::current_core]
	set_property vendor_display_name 	{Paderborn University - Computer Engineering Group} [ipx::current_core]
	set_property description 			{ReconOS Library} 		[ipx::current_core]

	# Set libraries
	foreach {lib} $libs {
                puts "\[RDK\] adding subcore $lib"
		ipx::add_subcore $lib [ipx::get_file_groups xilinx_anylanguagesynthesis -of_objects [ipx::current_core]]
	}
	
	set_property core_revision 1 [ipx::current_core]
	ipx::create_xgui_files [ipx::current_core]
	ipx::update_checksums [ipx::current_core]
	ipx::save_core [ipx::current_core]
	puts "\[RDK\] After save_core"
}

#
# MAIN
#

set ip_repo "pcores"
set temp_dir "/tmp/reconos_tmp/"

create_project -force managed_ip_project $temp_dir/managed_ip_project -part xc7z020clg400-1 -ip
set_property  ip_repo_paths  $ip_repo [current_project]

create_fifo_interfaces $ip_repo 
load_fifo_interfaces   $ip_repo

import_pcore $ip_repo reconos_v3_01_a ""; # ReconOS Lib has to be imported first, so other IP can use it

update_ip_catalog ;#-rebuild -repo_path $ip_repo

import_pcore $ip_repo reconos_clock_v1_00_a  "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo reconos_fifo_async_v1_00_a ""
import_pcore $ip_repo reconos_fifo_sync_v1_00_a ""
import_pcore $ip_repo reconos_hwt_idle_v1_00_a ""
import_pcore $ip_repo reconos_memif_arbiter_v1_00_a "cs.upb.de:reconos:reconos:3.01.a"
import_pcore $ip_repo reconos_memif_memory_controller_v1_00_a "cs.upb.de:reconos:reconos:3.01.a xilinx.com:ip:axi_master_burst:2.0"
import_pcore $ip_repo reconos_memif_mmu_microblaze_v1_00_a ""
import_pcore $ip_repo reconos_memif_mmu_zynq_v1_00_a "cs.upb.de:reconos:reconos:3.01.a"
import_pcore $ip_repo reconos_osif_intc_v1_00_a "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo reconos_osif_v1_00_a "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo reconos_proc_control_v1_00_a "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo timer_v1_00_a "xilinx.com:ip:axi_lite_ipif:3.0" 

# create empty list
set hwt_list [list]

# add all hardware threads to the list (substitute dots in version string with underscores)
<<generate for SLOTS>>
lappend hwt_list <<HwtCoreName>>_v[string map {. _} "<<HwtCoreVersion>>"]
<<end generate>>

# make the elements of the list unique, i.e. remove duplicates
set hwt_list [lsort -unique $hwt_list]

# now import all hardware threads exactly once
foreach hwt $hwt_list {
    import_pcore $ip_repo $hwt "cs.upb.de:reconos:reconos:3.01.a"
}

#
# AXI and other ports are automatically recognized, but our FIFO interfaces are not.
# Therefore we have to add them manually.
#
set ip_name "reconos_memif_arbiter_v1_00_a"

<<generate for SLOTS>>
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt_<<Id>>" "MEMIF_Mem2Hwt_<<Id>>_In_Full" "MEMIF_Mem2Hwt_<<Id>>_In_Data" "MEMIF_Mem2Hwt_<<Id>>_In_WE" "master"
add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem_<<Id>>" "MEMIF_Hwt2Mem_<<Id>>_In_Empty" "MEMIF_Hwt2Mem_<<Id>>_In_Data" "MEMIF_Hwt2Mem_<<Id>>_In_RE" "master"
<<end generate>>                      
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt_OUT" "MEMIF_Mem2Hwt_Out_Full" "MEMIF_Mem2Hwt_Out_Data" "MEMIF_Mem2Hwt_Out_WE" "slave"
add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem_OUT" "MEMIF_Hwt2Mem_Out_Empty" "MEMIF_Hwt2Mem_Out_Data" "MEMIF_Hwt2Mem_Out_RE" "slave"

set ip_name "reconos_fifo_async_v1_00_a"
add_fifo_m_interface $ip_repo/$ip_name "FIFO_M" "FIFO_M_Full" "FIFO_M_Data" "FIFO_M_WE" "slave"
add_fifo_s_interface $ip_repo/$ip_name "FIFO_S" "FIFO_S_Empty" "FIFO_S_Data" "FIFO_S_RE" "slave"

set ip_name "reconos_fifo_sync_v1_00_a"
add_fifo_m_interface $ip_repo/$ip_name "FIFO_M" "FIFO_M_Full" "FIFO_M_Data" "FIFO_M_WE" "slave"
add_fifo_s_interface $ip_repo/$ip_name "FIFO_S" "FIFO_S_Empty" "FIFO_S_Data" "FIFO_S_RE" "slave"

set ip_name "reconos_memif_memory_controller_v1_00_a"
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt_In" "MEMIF_Mem2Hwt_In_Full" "MEMIF_Mem2Hwt_In_Data" "MEMIF_Mem2Hwt_In_WE" "master"
add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem_In" "MEMIF_Hwt2Mem_In_Empty" "MEMIF_Hwt2Mem_In_Data" "MEMIF_Hwt2Mem_In_RE" "master"

# Interface definitions for "reconos_memif_mmu_microblaze_v1_00_a" may be wrong!
set ip_name "reconos_memif_mmu_microblaze_v1_00_a"
add_fifo_s_interface $ip_repo/$ip_name "CTRL_FIFO_In" "CTRL_FIFO_In_Empty" "CTRL_FIFO_In_Data" "CTRL_FIFO_In_RE" "master"
add_fifo_s_interface $ip_repo/$ip_name "CTRL_FIFO_Out" "CTRL_FIFO_Out_Empty" "CTRL_FIFO_Out_Data" "CTRL_FIFO_Out_RE" "slave"
add_fifo_s_interface $ip_repo/$ip_name "CTRL_FIFO_Mmu" "CTRL_FIFO_Mmu_Empty" "CTRL_FIFO_Mmu_Data" "CTRL_FIFO_Mmu_RE" "slave"
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_FIFO_Mmu" "MEMIF_FIFO_Mmu_Full" "MEMIF_FIFO_Mmu_Data" "MEMIF_FIFO_Mmu_WE" "slave"

set ip_name "reconos_memif_mmu_zynq_v1_00_a"
add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem_In" "MEMIF_Hwt2Mem_In_Empty" "MEMIF_Hwt2Mem_In_Data" "MEMIF_Hwt2Mem_In_RE" "master"
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt_In" "MEMIF_Mem2Hwt_In_Full" "MEMIF_Mem2Hwt_In_Data" "MEMIF_Mem2Hwt_In_WE" "master"

add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem_Out" "MEMIF_Hwt2Mem_Out_Empty" "MEMIF_Hwt2Mem_Out_Data" "MEMIF_Hwt2Mem_Out_RE" "slave"
add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt_Out" "MEMIF_Mem2Hwt_Out_Full" "MEMIF_Mem2Hwt_Out_Data" "MEMIF_Mem2Hwt_Out_WE" "slave"


foreach hwt $hwt_list {
    set ip_name $hwt
    add_fifo_s_interface $ip_repo/$ip_name "OSIF_Sw2Hw" "OSIF_Sw2Hw_Empty" "OSIF_Sw2Hw_Data" "OSIF_Sw2Hw_RE" "master"
    add_fifo_m_interface $ip_repo/$ip_name "OSIF_Hw2Sw" "OSIF_Hw2Sw_Full" "OSIF_Hw2Sw_Data" "OSIF_Hw2Sw_WE" "master"
    
    add_fifo_m_interface $ip_repo/$ip_name "MEMIF_Hwt2Mem" "MEMIF_Hwt2Mem_Full" "MEMIF_Hwt2Mem_Data" "MEMIF_Hwt2Mem_WE" "master"
    add_fifo_s_interface $ip_repo/$ip_name "MEMIF_Mem2Hwt" "MEMIF_Mem2Hwt_Empty" "MEMIF_Mem2Hwt_Data" "MEMIF_Mem2Hwt_RE" "master"    
}

set ip_name "reconos_osif_v1_00_a"
<<generate for SLOTS>>
add_fifo_m_interface $ip_repo/$ip_name "OSIF_Sw2Hw_<<Id>>" "OSIF_Sw2Hw_<<Id>>_In_Full"  "OSIF_Sw2Hw_<<Id>>_In_Data" "OSIF_Sw2Hw_<<Id>>_In_WE" "master"
add_fifo_s_interface $ip_repo/$ip_name "OSIF_Hw2Sw_<<Id>>" "OSIF_Hw2Sw_<<Id>>_In_Empty" "OSIF_Hw2Sw_<<Id>>_In_Data" "OSIF_Hw2Sw_<<Id>>_In_RE" "master"
<<end generate>>                      

# We need to mark the clock output of reconos_clock_v1_00_a as a clock output
set ip_name "reconos_clock_v1_00_a"
add_clock_interface $ip_repo/$ip_name "CLOCK" "CLK0_Out"


close_project
file delete -force $temp_dir

exit
