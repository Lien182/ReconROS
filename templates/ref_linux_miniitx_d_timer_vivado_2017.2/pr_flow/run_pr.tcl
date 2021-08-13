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
set xboard        "miniitx"

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

<<generate for SLOTS(Reconfigurable == True)>>
set rp<<Id>> "design_1_slot_<<Id>>_0"
set rp<<Id>>_inst "design_1_i/slot_<<Id>>"
set rm_variants($rp<<Id>>) "<<=generate for THREADS=>> <<Name>>_<<SlotId>> <<=end generate=>>"
<<end generate>>

########################################################################
### RM Configurations (Valid combinations of RM variants)
### 1. Define initial configuration: rm_config(initial)
### 2. Define additional configurations: rm_config(xyz)
########################################################################

<<generate for THREADS>>
<<RMConfiguration>>
<<end generate>>

########################################################################
### Task / flow portion
########################################################################
# Build the designs

open_project $srcDir/myReconOS.xpr
set_property synth_checkpoint_mode None [get_files  $srcDir/myReconOS.srcs/sources_1/bd/design_1/design_1.bd]
generate_target all [get_files  $srcDir/myReconOS.srcs/sources_1/bd/design_1/design_1.bd]
export_ip_user_files -of_objects [get_files  $srcDir/myReconOS.srcs/sources_1/bd/design_1/design_1.bd] -no_script -sync -force -quiet

source ./pr_flow/advanced_settings.tcl
source $tclDir/run.tcl

exit ;#uncomment if running in batch mode
