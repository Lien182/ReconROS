# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "ASYNC_SPI_CLK" -parent ${Page_0}
  ipgui::add_param $IPINST -name "ASYNC_TRIG" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CMD_MEM_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_OF_SDI" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SDO_MEM_ADDRESS_WIDTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.ASYNC_SPI_CLK { PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to update ASYNC_SPI_CLK when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ASYNC_SPI_CLK { PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to validate ASYNC_SPI_CLK
	return true
}

proc update_PARAM_VALUE.ASYNC_TRIG { PARAM_VALUE.ASYNC_TRIG } {
	# Procedure called to update ASYNC_TRIG when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ASYNC_TRIG { PARAM_VALUE.ASYNC_TRIG } {
	# Procedure called to validate ASYNC_TRIG
	return true
}

proc update_PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH { PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to update CMD_MEM_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH { PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to validate CMD_MEM_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to update DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to validate DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to update NUM_OF_SDI when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to validate NUM_OF_SDI
	return true
}

proc update_PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH { PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to update SDO_MEM_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH { PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to validate SDO_MEM_ADDRESS_WIDTH
	return true
}


proc update_MODELPARAM_VALUE.ASYNC_SPI_CLK { MODELPARAM_VALUE.ASYNC_SPI_CLK PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ASYNC_SPI_CLK}] ${MODELPARAM_VALUE.ASYNC_SPI_CLK}
}

proc update_MODELPARAM_VALUE.ASYNC_TRIG { MODELPARAM_VALUE.ASYNC_TRIG PARAM_VALUE.ASYNC_TRIG } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ASYNC_TRIG}] ${MODELPARAM_VALUE.ASYNC_TRIG}
}

proc update_MODELPARAM_VALUE.CMD_MEM_ADDRESS_WIDTH { MODELPARAM_VALUE.CMD_MEM_ADDRESS_WIDTH PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CMD_MEM_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.CMD_MEM_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.SDO_MEM_ADDRESS_WIDTH { MODELPARAM_VALUE.SDO_MEM_ADDRESS_WIDTH PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SDO_MEM_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.SDO_MEM_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.DATA_WIDTH { MODELPARAM_VALUE.DATA_WIDTH PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DATA_WIDTH}] ${MODELPARAM_VALUE.DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.NUM_OF_SDI { MODELPARAM_VALUE.NUM_OF_SDI PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_OF_SDI}] ${MODELPARAM_VALUE.NUM_OF_SDI}
}

