# TCK (时钟) - 引脚 AC14
set_property PACKAGE_PIN AC14 [get_ports jtag_tck_0]
set_property IOSTANDARD LVCMOS33 [get_ports jtag_tck_0]
set_property PACKAGE_PIN AF18 [get_ports resetn_0]
set_property IOSTANDARD LVCMOS33 [get_ports resetn_0]

#set_property PACKAGE_PIN F9 [get_ports clk_in1_p_0]
#set_property IOSTANDARD DIFF_SSTL15 [get_ports clk_in1_p_0]
# 强制忽略时钟布线检查（因为 JTAG 通常不走专用时钟引脚）
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_ports jtag_tck_0]]
set_property PACKAGE_PIN AE12 [get_ports rx_0]
set_property IOSTANDARD LVCMOS33 [get_ports rx_0]
set_property PACKAGE_PIN AF12 [get_ports tx_0]
set_property IOSTANDARD LVCMOS33 [get_ports tx_0]
# TMS (模式选择) - 引脚 AC13
set_property PACKAGE_PIN AC13 [get_ports jtag_tms_0]
set_property IOSTANDARD LVCMOS33 [get_ports jtag_tms_0]
set_property PULLUP TRUE [get_ports jtag_tms_0]

# TDI (数据输入) - 引脚 AC12
set_property PACKAGE_PIN AC12 [get_ports jtag_tdi_0]
set_property IOSTANDARD LVCMOS33 [get_ports jtag_tdi_0]
set_property PULLUP TRUE [get_ports jtag_tdi_0]

# TDO (数据输出) - 引脚 AB12
set_property PACKAGE_PIN AB12 [get_ports jtag_tdo_0]
set_property IOSTANDARD LVCMOS33 [get_ports jtag_tdo_0]
# 增强 TDO 的驱动能力，确保信号能穿透杜邦线到达 ESP-Prog
set_property SLEW FAST [get_ports jtag_tdo_0]
set_property DRIVE 12 [get_ports jtag_tdo_0]
set_property PULLUP TRUE [get_ports jtag_tdo_0]