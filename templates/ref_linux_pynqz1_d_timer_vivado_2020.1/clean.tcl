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
#   description:  This TCL script cleans an Vivado project from temporary
#                 and automatically created files. 
#
# ======================================================================

proc get_cpu_core_count {} {
    global tcl_platform env
    switch ${tcl_platform(platform)} {
	"windows" { 
	    return $env(NUMBER_OF_PROCESSORS)       
	}

	"unix" {
	    if {![catch {open "/proc/cpuinfo"} f]} {
		set cores [regexp -all -line {^processor\s} [read $f]]
		close $f
		if {$cores > 0} {
		    return $cores
		}
	    }
	}

	"Darwin" {
	    if {![catch {exec $sysctl -n "hw.ncpu"} cores]} {
		return $cores
	    }
	}

	default {
	    puts "Unknown System"
	    return 1
	}
    }
}

proc reconos_clean_project {} {
    open_project myReconOS.xpr
    
    # Some TCL commands to reduce 
    reset_project
    config_ip_cache -clear_local_cache
    config_ip_cache -clear_output_repo

    close_project
}


#
# MAIN
#

reconos_clean_project
exit
