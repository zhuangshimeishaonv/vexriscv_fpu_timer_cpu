# Repair pin-level JTAG connections in the copied FPU timer project and
# validate/generate synthesis targets without regenerating MIG simulation files.

set xpr {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
open_project $xpr
set_property ip_repo_paths [list \
  {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/ip_repo} \
  {C:/Users/minga/Desktop/ri} \
  {E:/mybishe/4.18/ip_repo} \
] [current_project]
update_ip_catalog -rebuild
open_bd_design [get_files design_1.bd]

proc connect_port_to_pin_if_needed {port_name pin_name} {
    set port [get_bd_ports -quiet $port_name]
    set pin  [get_bd_pins  -quiet $pin_name]
    if {[llength $port] == 0 || [llength $pin] == 0} {
        puts "WARNING: missing $port_name or $pin_name"
        return
    }
    if {[llength [get_bd_nets -quiet -of_objects $pin]] == 0} {
        connect_bd_net $port $pin
    }
}

connect_port_to_pin_if_needed jtag_tms_0 VexRiscvAxi4_Wrapper_0/jtag_tms
connect_port_to_pin_if_needed jtag_tdi_0 VexRiscvAxi4_Wrapper_0/jtag_tdi
connect_port_to_pin_if_needed jtag_tck_0 VexRiscvAxi4_Wrapper_0/jtag_tck
connect_port_to_pin_if_needed jtag_tdo_0 VexRiscvAxi4_Wrapper_0/jtag_tdo

validate_bd_design
save_bd_design
generate_target synthesis [get_files design_1.bd]
make_wrapper -files [get_files design_1.bd] -top -force
set wrapper [lindex [glob -nocomplain {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.gen/sources_1/bd/design_1/hdl/design_1_wrapper.v}] 0]
if {$wrapper ne ""} {
  add_files -norecurse $wrapper
  set_property top design_1_wrapper [current_fileset]
}
update_compile_order -fileset sources_1
close_project
