# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "C_S_AXI_ADDR_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "C_S_AXI_DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_PATHS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_SNAPSHOTS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_X_LEN" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MAX_Y_LEN" -parent ${Page_0}


}

proc update_PARAM_VALUE.C_S_AXI_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_ADDR_WIDTH } {
	# Procedure called to update C_S_AXI_ADDR_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_ADDR_WIDTH { PARAM_VALUE.C_S_AXI_ADDR_WIDTH } {
	# Procedure called to validate C_S_AXI_ADDR_WIDTH
	return true
}

proc update_PARAM_VALUE.C_S_AXI_DATA_WIDTH { PARAM_VALUE.C_S_AXI_DATA_WIDTH } {
	# Procedure called to update C_S_AXI_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.C_S_AXI_DATA_WIDTH { PARAM_VALUE.C_S_AXI_DATA_WIDTH } {
	# Procedure called to validate C_S_AXI_DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.MAX_PATHS { PARAM_VALUE.MAX_PATHS } {
	# Procedure called to update MAX_PATHS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_PATHS { PARAM_VALUE.MAX_PATHS } {
	# Procedure called to validate MAX_PATHS
	return true
}

proc update_PARAM_VALUE.MAX_SNAPSHOTS { PARAM_VALUE.MAX_SNAPSHOTS } {
	# Procedure called to update MAX_SNAPSHOTS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_SNAPSHOTS { PARAM_VALUE.MAX_SNAPSHOTS } {
	# Procedure called to validate MAX_SNAPSHOTS
	return true
}

proc update_PARAM_VALUE.MAX_X_LEN { PARAM_VALUE.MAX_X_LEN } {
	# Procedure called to update MAX_X_LEN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_X_LEN { PARAM_VALUE.MAX_X_LEN } {
	# Procedure called to validate MAX_X_LEN
	return true
}

proc update_PARAM_VALUE.MAX_Y_LEN { PARAM_VALUE.MAX_Y_LEN } {
	# Procedure called to update MAX_Y_LEN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MAX_Y_LEN { PARAM_VALUE.MAX_Y_LEN } {
	# Procedure called to validate MAX_Y_LEN
	return true
}


proc update_MODELPARAM_VALUE.C_S_AXI_ADDR_WIDTH { MODELPARAM_VALUE.C_S_AXI_ADDR_WIDTH PARAM_VALUE.C_S_AXI_ADDR_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_S_AXI_ADDR_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_ADDR_WIDTH}
}

proc update_MODELPARAM_VALUE.C_S_AXI_DATA_WIDTH { MODELPARAM_VALUE.C_S_AXI_DATA_WIDTH PARAM_VALUE.C_S_AXI_DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.C_S_AXI_DATA_WIDTH}] ${MODELPARAM_VALUE.C_S_AXI_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.MAX_X_LEN { MODELPARAM_VALUE.MAX_X_LEN PARAM_VALUE.MAX_X_LEN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_X_LEN}] ${MODELPARAM_VALUE.MAX_X_LEN}
}

proc update_MODELPARAM_VALUE.MAX_Y_LEN { MODELPARAM_VALUE.MAX_Y_LEN PARAM_VALUE.MAX_Y_LEN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_Y_LEN}] ${MODELPARAM_VALUE.MAX_Y_LEN}
}

proc update_MODELPARAM_VALUE.MAX_SNAPSHOTS { MODELPARAM_VALUE.MAX_SNAPSHOTS PARAM_VALUE.MAX_SNAPSHOTS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_SNAPSHOTS}] ${MODELPARAM_VALUE.MAX_SNAPSHOTS}
}

proc update_MODELPARAM_VALUE.MAX_PATHS { MODELPARAM_VALUE.MAX_PATHS PARAM_VALUE.MAX_PATHS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MAX_PATHS}] ${MODELPARAM_VALUE.MAX_PATHS}
}

