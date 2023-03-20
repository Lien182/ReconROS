<<reconos_preproc>>

open_project hls
set_top rt_imp
add_files reconos_calls.h -cflags "-m<<CPUARCHITECTURESIZE>>"
add_files reconos_thread.h -cflags "-m<<CPUARCHITECTURESIZE>>"
add_files [ glob *.cpp ] -cflags "-I/opt/ros/<<ROSDISTRIBUTION>>/include/ -m<<CPUARCHITECTURESIZE>> <<MSGINCLUDEDIR>>"
open_solution sol
set_part {<<PART>>}
create_clock -period <<CLKPRD>> -name default
source directives.tcl
csynth_design
export_design
exit