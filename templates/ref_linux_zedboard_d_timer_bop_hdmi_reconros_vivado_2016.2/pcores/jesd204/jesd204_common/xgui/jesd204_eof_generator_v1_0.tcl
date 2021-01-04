# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "DATA_PATH_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_OCTETS_PER_FRAME" -parent ${Page_0}


}

proc update_PARAM_VALUE.DATA_PATH_WIDTH { PARAM_VALUE.DATA_PATH_WIDTH } {
	# Procedure called to update DATA_PATH_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DATA_PATH_WIDTH { PARAM_VALUE.DATA_PATH_WIDTH } {
	# Procedure called to validate DATA_PATH_WIDTH
	return true
}

proc update_PARAM_VALUE.MAX_OCTETS_PER_FRAME { PARAM_VALUE.MAX_OCTETS_PER_FRAME } {
	# Procedure called to update MAX_OCTETS_PER_FRAME when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_OCTETS_PER_FRAME { PARAM_VALUE.MAX_OCTETS_PER_FRAME } {
	# Procedure called to validate MAX_OCTETS_PER_FRAME
	return true
}


proc update_MODELPARAM_VALUE.DATA_PATH_WIDTH { MODELPARAM_VALUE.DATA_PATH_WIDTH PARAM_VALUE.DATA_PATH_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DATA_PATH_WIDTH}] ${MODELPARAM_VALUE.DATA_PATH_WIDTH}
}

proc update_MODELPARAM_VALUE.MAX_OCTETS_PER_FRAME { MODELPARAM_VALUE.MAX_OCTETS_PER_FRAME PARAM_VALUE.MAX_OCTETS_PER_FRAME } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_OCTETS_PER_FRAME}] ${MODELPARAM_VALUE.MAX_OCTETS_PER_FRAME}
}

