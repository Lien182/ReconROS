<<reconos_preproc>>

###############################################################
###   Tcl Variables
###############################################################
####Define location for "Tcl" directory. Defaults to "./Tcl"
set tclHome "./pr_flow/tcl"
if {[file exists $tclHome]} {
   set tclDir $tclHome
} elseif {[file exists "./tcl"]} {
   set tclDir  "./tcl"
} else {
   error "ERROR: No valid location found for required Tcl scripts. Set \$tclDir in design.tcl to a valid location."
}
puts "Setting TCL dir to $tclDir"

####Source required Tcl Procs
source $tclDir/design_utils.tcl
source $tclDir/log_utils.tcl
source $tclDir/synth_utils.tcl
source $tclDir/impl_utils.tcl
source $tclDir/hd_floorplan_utils.tcl

source $tclDir/pr_utils.tcl

###############################################################
### Define Part, Package, Speedgrade 
###############################################################
set device       "xc7z020"
set package      "clg484"
set speed        "-1"
set part         $device$package$speed
check_part $part

###############################################################
###  Setup Variables
###############################################################
#set tclParams [list <param1> <value> <param2> <value> ... <paramN> <value>]
set tclParams [list hd.visual 1 \
              ]
              

<<reconos_preproc>>

###############################################################
###  Minimum settings required to run PR flow:
###  1. Specify flow steps
###  2. Define target board
###  3. Identify source directories
###  4. Define static module
###  5. Define RPs, and their RM variants
###############################################################
####flow control (1 = run step , 0 = skip step)
set run.topSynth       1 ;#synthesize static
set run.rmSynth        1 ;#synthesize RM variants
set run.prImpl         1 ;#implement each static + RM configuration
set run.prVerify       1 ;#verify RMs are compatible with static
set run.writeBitstream 1 ;#generate full and partial bitstreams

###############################################################
### Define target demo board
### Valid values are kcu105, vcu108, kcu116 and vcu118
### Select one only
###############################################################
set xboard        "zcu102"

###############################################################
###  Run Settings
###############################################################
####Input Directories
#set srcDir     "./Sources"
set srcDir     "."

set prjDir     "$srcDir/pcores/prj"

#not used, paths in advanced_settings point directly to relevant files
#set rtlDir     "$srcDir/hdl"
#set xdcDir     "$srcDir/xdc"
#set coreDir    "$srcDir/cores"
#set netlistDir "$srcDir/netlist"

####Output Directories
set synthDir  "./Synth"
set implDir   "./Implement"
set dcpDir    "./Checkpoint"
set bitDir    "./Bitstreams"

###############################################################
### Static Module Definition
###############################################################
set top "design_1_wrapper"

###############################################################
### RP & RM Definitions (Repeat for each RP)
### 1. Define Reconfigurable Partition (RP) name
### 2. Define corresponoding top level cell (added this to work with ReconOS-generated static design)
### 3. Associate Reconfigurable Modules (RMs) to the RP
###############################################################
#set rp1 "design_1_slot_0_0"
#set rp1_inst "design_1_i/slot_0"
#set rm_variants($rp1) "sortdemo_0 matrixmul_0"
#set rp2 "design_1_slot_1_0"
#set rp2_inst "design_1_i/slot_1"
#set rm_variants($rp2) "sortdemo_1 matrixmul_1"

<<generate for SLOTS(Reconfigurable == "true")>>
set rp<<Id>> "design_1_slot_<<Id>>_0"
set rp<<Id>>_inst "design_1_i/slot_<<Id>>"
set rm_variants($rp<<Id>>) "<<Threadnames>>"
<<end generate>>

########################################################################
### RM Configurations (Valid combinations of RM variants)
### 1. Define initial configuration: rm_config(initial)
### 2. Define additional configurations: rm_config(xyz)
########################################################################
#set module1_variant1 "sortdemo_0"
#set module2_variant1 "sortdemo_1"
#set rm_config(initial)   "$rp1 $rp1_inst $module1_variant1 $rp2 $rp2_inst $module2_variant1"
#set module1_variant2 "matrixmul_0"
#set module2_variant2 "matrixmul_1"
#set rm_config(reconfig1) "$rp1 $rp1_inst $module1_variant2 $rp2 $rp2_inst $module2_variant2"

<<generate for THREADS>>
<<RMConfiguration>>
<<end generate>>

########################################################################
### Task / flow portion
########################################################################
# Build the designs
#source ./pr_flow/advanced_settings.tcl
#source $tclDir/run.tcl

#exit ;#uncomment if running in batch mode



####flow control
set run.rmSynth        1
set run.prImpl         1
set run.prVerify       1
set run.writeBitstream 1
set run.flatImpl       0

####Report and DCP controls - values: 0-required min; 1-few extra; 2-all
set verbose      1
set dcpLevel     1

####Output Directories
set synthDir  "./Synth"
set implDir   "./Implement"
set dcpDir    "./Checkpoint"
set bitDir    "./Bitstreams"

####Input Directories, not used in this demo. Paths directly point to the files in the build.hw and src folders.
set srcDir     "."
set xdcDir     "./constr"
set rtlDir     "./src/application"
#set ipDir      ""
set prjDir     "$srcDir/pcores/prj"

###############################################################
### Top Definition
###############################################################
set top "design_1_wrapper"
set static "static"
add_module $static
set_attribute module $static moduleName      $top
set_attribute module $static top_level       1
set_attribute module $static synthCheckpoint ./build.hw/myReconOS.runs/synth_1/design_1_wrapper.dcp

set_attribute module $static ipRepo          ./pcores
set_attribute module $static bd              ./myReconOS.srcs/sources_1/bd/design_1/design_1.bd
set_attribute module $static vhdl            [list ./myReconOS.srcs/sources_1/bd/design_1/hdl/design_1_wrapper.vhd xil_defaultLib]
set_attribute module $static synth           ${run.topSynth}

####################################################################
### RP Module Definitions
####################################################################
foreach rp [array names rm_variants] {
  foreach rm $rm_variants($rp) {
    set variant $rm
    add_module $variant
    set_attribute module $variant moduleName   $rp
    set_attribute module $variant prj          $prjDir/$variant.prj
    set_attribute module $variant synth        ${run.rmSynth}
  }
}

########################################################################
### Configuration (Implementation) Definition - Replicate for each Config
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
  #set_attribute impl $config bitstream_options [list "-bin_file"]
  #set_attribute impl $config bitstream_settings [list <settings_go_here>]
}

####################################################################
### Create Flat implementation run 
####################################################################

add_implementation Flat
set_attribute impl Flat top          $top
set_attribute impl Flat implXDC      [list ./build.hw/myReconOS.srcs/constrs_1/new/design_1_pblocks.xdc 
                                     ]
set_attribute impl Flat partitions   $partition_list
set_attribute impl Flat impl         ${run.flatImpl}

########################################################################
### Task / flow portion
########################################################################
source $tclDir/run.tcl

exit
