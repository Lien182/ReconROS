# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "ASYNC_SPI_CLK" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CMD_FIFO_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "ID" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MM_IF_TYPE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_OFFLOAD" -parent ${Page_0}
  ipgui::add_param $IPINST -name "NUM_OF_SDI" -parent ${Page_0}
  ipgui::add_param $IPINST -name "OFFLOAD0_CMD_MEM_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "OFFLOAD0_SDO_MEM_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SDI_FIFO_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SDO_FIFO_ADDRESS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "UP_ADDRESS_WIDTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.ASYNC_SPI_CLK { PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to update ASYNC_SPI_CLK when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ASYNC_SPI_CLK { PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to validate ASYNC_SPI_CLK
	return true
}

proc update_PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH { PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH } {
	# Procedure called to update CMD_FIFO_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH { PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH } {
	# Procedure called to validate CMD_FIFO_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to update DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.DATA_WIDTH { PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to validate DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.ID { PARAM_VALUE.ID } {
	# Procedure called to update ID when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ID { PARAM_VALUE.ID } {
	# Procedure called to validate ID
	return true
}

proc update_PARAM_VALUE.MM_IF_TYPE { PARAM_VALUE.MM_IF_TYPE } {
	# Procedure called to update MM_IF_TYPE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MM_IF_TYPE { PARAM_VALUE.MM_IF_TYPE } {
	# Procedure called to validate MM_IF_TYPE
	return true
}

proc update_PARAM_VALUE.NUM_OFFLOAD { PARAM_VALUE.NUM_OFFLOAD } {
	# Procedure called to update NUM_OFFLOAD when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_OFFLOAD { PARAM_VALUE.NUM_OFFLOAD } {
	# Procedure called to validate NUM_OFFLOAD
	return true
}

proc update_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to update NUM_OF_SDI when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.NUM_OF_SDI { PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to validate NUM_OF_SDI
	return true
}

proc update_PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH { PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to update OFFLOAD0_CMD_MEM_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH { PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to validate OFFLOAD0_CMD_MEM_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH { PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to update OFFLOAD0_SDO_MEM_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH { PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to validate OFFLOAD0_SDO_MEM_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH { PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH } {
	# Procedure called to update SDI_FIFO_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH { PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH } {
	# Procedure called to validate SDI_FIFO_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH { PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH } {
	# Procedure called to update SDO_FIFO_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH { PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH } {
	# Procedure called to validate SDO_FIFO_ADDRESS_WIDTH
	return true
}

proc update_PARAM_VALUE.UP_ADDRESS_WIDTH { PARAM_VALUE.UP_ADDRESS_WIDTH } {
	# Procedure called to update UP_ADDRESS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.UP_ADDRESS_WIDTH { PARAM_VALUE.UP_ADDRESS_WIDTH } {
	# Procedure called to validate UP_ADDRESS_WIDTH
	return true
}


proc update_MODELPARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH { MODELPARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.CMD_FIFO_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH { MODELPARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.SDO_FIFO_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH { MODELPARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.SDI_FIFO_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.MM_IF_TYPE { MODELPARAM_VALUE.MM_IF_TYPE PARAM_VALUE.MM_IF_TYPE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MM_IF_TYPE}] ${MODELPARAM_VALUE.MM_IF_TYPE}
}

proc update_MODELPARAM_VALUE.UP_ADDRESS_WIDTH { MODELPARAM_VALUE.UP_ADDRESS_WIDTH PARAM_VALUE.UP_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.UP_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.UP_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.ASYNC_SPI_CLK { MODELPARAM_VALUE.ASYNC_SPI_CLK PARAM_VALUE.ASYNC_SPI_CLK } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ASYNC_SPI_CLK}] ${MODELPARAM_VALUE.ASYNC_SPI_CLK}
}

proc update_MODELPARAM_VALUE.NUM_OFFLOAD { MODELPARAM_VALUE.NUM_OFFLOAD PARAM_VALUE.NUM_OFFLOAD } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_OFFLOAD}] ${MODELPARAM_VALUE.NUM_OFFLOAD}
}

proc update_MODELPARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH { MODELPARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.OFFLOAD0_CMD_MEM_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH { MODELPARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH}] ${MODELPARAM_VALUE.OFFLOAD0_SDO_MEM_ADDRESS_WIDTH}
}

proc update_MODELPARAM_VALUE.ID { MODELPARAM_VALUE.ID PARAM_VALUE.ID } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ID}] ${MODELPARAM_VALUE.ID}
}

proc update_MODELPARAM_VALUE.DATA_WIDTH { MODELPARAM_VALUE.DATA_WIDTH PARAM_VALUE.DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.DATA_WIDTH}] ${MODELPARAM_VALUE.DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.NUM_OF_SDI { MODELPARAM_VALUE.NUM_OF_SDI PARAM_VALUE.NUM_OF_SDI } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.NUM_OF_SDI}] ${MODELPARAM_VALUE.NUM_OF_SDI}
}

