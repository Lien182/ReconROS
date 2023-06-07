# findFiles
# basedir - the directory to start looking in
# pattern - A pattern, as defined by the glob command, that the files must match
proc findFiles { basedir pattern } {

    # Fix the directory name, this ensures the directory name is in the
    # native format for the platform and contains a final directory seperator
    set basedir [string trimright [file join [file normalize $basedir] { }]]
    set fileList {}

    # Look in the current directory for matching files, -type {f r}
    # means ony readable normal files are looked at, -nocomplain stops
    # an error being thrown if the returned list is empty
    foreach fileName [glob -nocomplain -type {f r} -path $basedir $pattern] {
        lappend fileList $fileName
    }

    # Now look for any sub direcories in the current directory
    foreach dirName [glob -nocomplain -type {d  r} -path $basedir *] {
        # Recusively call the routine on the sub directory and append any
        # new files to the results
        set subDirList [findFiles $dirName $pattern]
        if { [llength $subDirList] > 0 } {
            foreach subDirFile $subDirList {
                lappend fileList $subDirFile
            }
        }
    }
    return $fileList
 }


###############################################################
### Advanced Settings
###############################################################
# Load utilities
#Define location for "Tcl" directory. Defaults to "./tcl_HD"
if {[file exists "./pr_flow/Tcl_HD"]} { 
   set tclDir  "./pr_flow/Tcl_HD"
} else {
   error "ERROR: No valid location found for required Tcl scripts. Set \$tclDir in design.tcl to a valid location."
}
puts "Setting TCL dir to $tclDir"

####Source required Tcl Procs
source $tclDir/design_utils.tcl
source $tclDir/log_utils.tcl
source $tclDir/synth_utils.tcl
source $tclDir/impl_utils.tcl
source $tclDir/hd_utils.tcl
source $tclDir/pr_utils.tcl

###############################################################
### Board Settings
### -Board: default device, package and speed for selected board
###############################################################
switch $xboard {
vcu108 {
 set device       "xcvu095"
 set package      "-ffva2104"
 set speed        "-2-e"
}
kcu116 {
 set device       "xcku5p"
 set package      "-ffvb676"
 set speed        "-2-e"
}
vcu118 {
 set device       "xcvu9p"
 set package      "-flga2104"
 set speed        "-2l-e"
}
zcu102 {
 set device       "xczu9eg"
 set package      "-ffvb1156"
 set speed        "-2-e"
 set board        "xilinx.com:zcu102:3.2"
}
zcu104 {
 set device       "xczu7ev"
 set package      "-ffvc1156"
 set speed        "-2-e"
 set board        "xilinx.com:zcu104:1.1"
}
default {
 #kcu105
 set device       "xcku040"
 set package      "-ffva1156"
 set speed        "-2-e"
}
}
set part         $device$package$speed
check_part $part

###############################################################
###  Run Settings
###############################################################
####Report and DCP controls - values: 0-required min; 1-few extra; 2-all
set verbose      1
set dcpLevel     1

###############################################################
### Static Module Definition
###############################################################
set static "Static"
add_module $static
set_attribute module $static moduleName    $top
set_attribute module $static top_level     1
#if {$xboard == "ac701"} {
#   set_attribute module $static vlog          [list [glob $rtlDir/$top/$xboard/*.v]]
#} else {
#   set_attribute module $static vlog          [list [glob $rtlDir/$top/*.v]]
#}
#set_attribute module $static synthCheckpoint ./myReconOS.runs/synth_1/design_1_wrapper.dcp
set_attribute module $static ipRepo          ./pcores
set_attribute module $static bd              ./myReconOS.srcs/sources_1/bd/design_1/design_1.bd
set_attribute module $static vhdl            [list ./myReconOS.gen/sources_1/bd/design_1/hdl/design_1_wrapper.vhd xil_defaultLib]
set_attribute module $static synth           ${run.topSynth}

####################################################################
### RP Module Definitions
####################################################################
foreach rp [array names rm_variants] {
  foreach rm $rm_variants($rp) {
    set variant $rm
    add_module $variant
    set_attribute module $variant moduleName   $rp
    #set_attribute module $variant vlog         [list $rtlDir/$variant/$variant.v]
    regsub {_0$} $variant {} variantnew
    set ippath ./pcores/rt_${variantnew}_v1_00_a/ip/
    puts $ippath
    set_attribute module $variant ip           [findFiles $ippath *.xci]
    puts [get_attribute module $variant ip]
    set_attribute module $variant prj          $prjDir/$variant.prj
    set_attribute module $variant synth        ${run.rmSynth}
  }
}

########################################################################
### Configuration (Implementation) Definition 
########################################################################
foreach cfg_name [array names rm_config] {
  if {$cfg_name=="initial"} {set state "implement"} else {set state "import"}
    
  set config "Config"
  set partition_list [list [list $static $top $state]]

  foreach {rp rp_inst rm_variant} $rm_config($cfg_name) {
    #set module_inst inst_${rp}
    set module_inst ${rp_inst}
    set config "${config}_${rm_variant}"
    set partition [list $rm_variant $module_inst implement]
    lappend partition_list $partition
  }
 set config "${config}_${state}"
  
  add_implementation $config
  set_attribute impl $config top             $top
  #set_attribute impl $config implXDC         [list $xdcDir/${top}_$xboard.xdc]
  set_attribute impl $config implXDC         [list pr_flow/pblocks.xdc]

  set_attribute impl $config partitions      $partition_list
  set_attribute impl $config pr.impl         1 
  set_attribute impl $config impl            ${run.prImpl} 
  set_attribute impl $config verify     	    ${run.prVerify} 
  set_attribute impl $config bitstream  	    ${run.writeBitstream} 
  #bitstream_options are given to write_bitstream command, bitstream_settings are set as property
  set_attribute impl $config bitstream_options [list "-bin_file"]
  #set_attribute impl $config bitstream_settings [list <settings_go_here>]
}
