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

proc load_fifo_interfaces {repo_path} {
	#
	# Opens interface definitions
	#
	ipx::open_ipxact_file $repo_path/FIFO_M.xml
	ipx::open_ipxact_file $repo_path/FIFO_S.xml
	ipx::open_ipxact_file $repo_path/FIFO64_M.xml
	ipx::open_ipxact_file $repo_path/FIFO64_S.xml
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
		ipx::infer_core -verbose -set_current true -as_library false -vendor cs.upb.de -taxonomy /ReconOS  $reconos_pcore_dir/$reconos_pcore
	}

	puts "\[RDK\] After infer_core"
	set_property vendor 				cs.upb.de 				[ipx::current_core]
	set_property library 				reconos 				[ipx::current_core]
	set_property company_url 			http://www.reconos.de 	[ipx::current_core]
	set_property vendor_display_name 	{Paderborn University - Computer Engineering Group} [ipx::current_core]
	set_property description 			{ReconOS Library} 		[ipx::current_core]

	# Set libraries
	foreach {lib} $libs {
                puts "\[RDK\] adding subcore $lib"
		ipx::add_subcore $lib [ipx::get_file_groups xilinx_anylanguagesynthesis -of_objects [ipx::current_core]]
	}

	set_property top $reconos_pcore_name [current_fileset]
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

create_project -force managed_ip_project $temp_dir/managed_ip_project -part xczu7ev-ffvc1156-2-e -ip
set_property  ip_repo_paths  $ip_repo [current_project]

# load IP-XACT definitions of FIFO interfaces (these are supplied with the template)
load_fifo_interfaces   $ip_repo

# make sure project is using automatic compile order before importing pcores without legacy .pao files
set_property source_mgmt_mode All [current_project]

import_pcore $ip_repo reconos_v3_01_a ""; # ReconOS Lib has to be imported first, so other IP can use it

update_ip_catalog ;#-rebuild -repo_path $ip_repo

import_pcore $ip_repo reconos_clock_v1_00_a  "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo reconos_fifo_async_v1_00_a ""
import_pcore $ip_repo reconos_fifo64_async_v1_00_a ""
import_pcore $ip_repo reconos_fifo_sync_v1_00_a ""
import_pcore $ip_repo reconos_fifo64_sync_v1_00_a ""
import_pcore $ip_repo reconos_hwt_idle_v1_00_a ""
import_pcore $ip_repo reconos_memif_arbiter_v1_00_a "cs.upb.de:reconos:reconos:3.01.a"
import_pcore $ip_repo reconos_memif_memory_controller_v1_00_a "cs.upb.de:reconos:reconos:3.01.a"
import_pcore $ip_repo reconos_memif_mmu_microblaze_v1_00_a ""
import_pcore $ip_repo reconos_memif_mmu_usp_v1_00_a "cs.upb.de:reconos:reconos:3.01.a"
import_pcore $ip_repo reconos_osif_intc_v1_00_a "xilinx.com:ip:axi_lite_ipif:3.0" 
import_pcore $ip_repo reconos_osif_v1_00_a "" 
import_pcore $ip_repo reconos_proc_control_v1_00_a 
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

close_project
file delete -force $temp_dir

exit
