# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "DEFAULT_CLK_DIV" -parent ${Page_0}
  ipgui::add_param $IPINST -name "DEFAULT_SPI_CFG" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_OF_CS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_OF_SDI" -parent ${Page_0}


}

proc update_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to update DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to validate DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.DEFAULT_CLK_DIV { PARAM_VALUE.DEFAULT_CLK_DIV } {
	# Procedure called to update DEFAULT_CLK_DIV when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DEFAULT_CLK_DIV { PARAM_VALUE.DEFAULT_CLK_DIV } {
	# Procedure called to validate DEFAULT_CLK_DIV
	return true
}

proc update_PARAM_VALUE.DEFAULT_SPI_CFG { PARAM_VALUE.DEFAULT_SPI_CFG } {
	# Procedure called to update DEFAULT_SPI_CFG when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DEFAULT_SPI_CFG { PARAM_VALUE.DEFAULT_SPI_CFG } {
	# Procedure called to validate DEFAULT_SPI_CFG
	return true
}

proc update_PARAM_VALUE.NUM_OF_CS { PARAM_VALUE.NUM_OF_CS } {
	# Procedure called to update NUM_OF_CS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_OF_CS { PARAM_VALUE.NUM_OF_CS } {
	# Procedure called to validate NUM_OF_CS
	return true
}

proc update_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to update NUM_OF_SDI when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to validate NUM_OF_SDI
	return true
}


proc update_MODELPARAM_VALUE.NUM_OF_CS { MODELPARAM_VALUE.NUM_OF_CS PARAM_VALUE.NUM_OF_CS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_OF_CS}] ${MODELPARAM_VALUE.NUM_OF_CS}
}

proc update_MODELPARAM_VALUE.DEFAULT_SPI_CFG { MODELPARAM_VALUE.DEFAULT_SPI_CFG PARAM_VALUE.DEFAULT_SPI_CFG } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DEFAULT_SPI_CFG}] ${MODELPARAM_VALUE.DEFAULT_SPI_CFG}
}

proc update_MODELPARAM_VALUE.DEFAULT_CLK_DIV { MODELPARAM_VALUE.DEFAULT_CLK_DIV PARAM_VALUE.DEFAULT_CLK_DIV } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DEFAULT_CLK_DIV}] ${MODELPARAM_VALUE.DEFAULT_CLK_DIV}
}

proc update_MODELPARAM_VALUE.DATA_WIDTH { MODELPARAM_VALUE.DATA_WIDTH PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DATA_WIDTH}] ${MODELPARAM_VALUE.DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.NUM_OF_SDI { MODELPARAM_VALUE.NUM_OF_SDI PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_OF_SDI}] ${MODELPARAM_VALUE.NUM_OF_SDI}
}

