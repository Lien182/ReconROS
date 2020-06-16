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

proc reconos_built_bitstream {} {
    # create bitstream
    open_project myReconOS.xpr
    launch_runs impl_1 -to_step write_bitstream -jobs [ expr [get_cpu_core_count] / 2 + 1]
    wait_on_run impl_1
    close_project
}


#
# MAIN
#

reconos_built_bitstream
exit
