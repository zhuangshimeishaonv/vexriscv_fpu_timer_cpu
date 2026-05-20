# Create an independent Vivado project from E:/mybishe/4.18 and replace the
# original VexRiscv CPU IP with the RV32IMF FPU + CSR timer CPU IP.
#
# Run:
#   E:/vivado/Vivado/2022.2/bin/vivado.bat -mode batch -source C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/scripts/create_project_from_4_18_replace_cpu.tcl -nojournal -log C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/create_fpu_timer_project.log
#
# Output:
#   C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr

set script_dir [file normalize [file dirname [info script]]]
set root_dir   [file normalize [file join $script_dir ..]]
set src_xpr    [file normalize {E:/mybishe/4.18/riaxiwithddr.xpr}]
set proj_dir   [file normalize [file join $root_dir vivado_project]]
set proj_name  "riaxiwithddr_fpu_timer"
set new_repo   [file normalize [file join $root_dir ip_repo]]
set old_repo_1 [file normalize {C:/Users/minga/Desktop/ri}]
set old_repo_2 [file normalize {E:/mybishe/4.18/ip_repo}]

proc one_or_empty {objects} {
    if {[llength $objects] == 0} { return "" }
    return [lindex $objects 0]
}

proc save_intf_peers {pin_name} {
    set pin [get_bd_intf_pins -quiet $pin_name]
    if {[llength $pin] == 0} { return "" }
    set net [one_or_empty [get_bd_intf_nets -quiet -of_objects $pin]]
    if {$net eq ""} { return "" }
    set peers ""
    foreach p [get_bd_intf_pins -quiet -of_objects $net] {
        if {$p ne $pin} { lappend peers $p }
    }
    foreach p [get_bd_intf_ports -quiet -of_objects $net] {
        lappend peers $p
    }
    return $peers
}

proc save_pin_net {pin_name} {
    set pin [get_bd_pins -quiet $pin_name]
    if {[llength $pin] == 0} { return "" }
    set net [one_or_empty [get_bd_nets -quiet -of_objects $pin]]
    if {$net eq ""} { return "" }
    return [get_property NAME $net]
}

proc reconnect_intf {peer_list pin_name} {
    if {$peer_list ne "" && [llength [get_bd_intf_pins -quiet $pin_name]] > 0} {
        foreach peer $peer_list {
            if {[llength [get_bd_intf_pins -quiet $peer]] > 0} {
                connect_bd_intf_net [get_bd_intf_pins $peer] [get_bd_intf_pins $pin_name]
            } elseif {[llength [get_bd_intf_ports -quiet $peer]] > 0} {
                connect_bd_intf_net [get_bd_intf_ports $peer] [get_bd_intf_pins $pin_name]
            }
        }
    }
}

proc reconnect_pin {net_name pin_name} {
    if {$net_name ne "" && [llength [get_bd_pins -quiet $pin_name]] > 0} {
        connect_bd_net -net $net_name [get_bd_pins $pin_name]
    }
}

open_project $src_xpr
save_project_as $proj_name $proj_dir -force
close_project

set new_xpr [file normalize [file join $proj_dir ${proj_name}.xpr]]
open_project $new_xpr

set repo_paths [list $new_repo]
foreach repo [list $old_repo_1 $old_repo_2] {
    if {[file exists $repo]} {
        lappend repo_paths $repo
    }
}
set_property ip_repo_paths $repo_paths [current_project]
update_ip_catalog -rebuild

set bd_files [get_files -quiet */design_1.bd]
if {[llength $bd_files] == 0} {
    set bd_files [get_files -quiet design_1.bd]
}
if {[llength $bd_files] == 0} {
    puts "ERROR: design_1.bd not found in copied project."
    exit 1
}
open_bd_design [lindex $bd_files 0]

set old_cell_name "VexRiscvAxi4_Wrapper_0"
if {[llength [get_bd_cells -quiet $old_cell_name]] == 0} {
    puts "ERROR: $old_cell_name not found in design_1.bd."
    exit 1
}

set ibus_peers  [save_intf_peers ${old_cell_name}/ibus]
set dbus_peers  [save_intf_peers ${old_cell_name}/dbus]
set jtag_peers  [save_intf_peers ${old_cell_name}/jtag]
set clk_net     [save_pin_net  ${old_cell_name}/clk]
set reset_net   [save_pin_net  ${old_cell_name}/reset]

delete_bd_objs [get_bd_cells $old_cell_name]
create_bd_cell -type ip -vlnv xilinx.com:user:VexRiscvAxi4FpuTimer_Wrapper:1.0 $old_cell_name

reconnect_intf $ibus_peers  ${old_cell_name}/ibus
reconnect_intf $dbus_peers  ${old_cell_name}/dbus
reconnect_intf $jtag_peers  ${old_cell_name}/jtag
reconnect_pin  $clk_net   ${old_cell_name}/clk
reconnect_pin  $reset_net ${old_cell_name}/reset

validate_bd_design
save_bd_design
generate_target all [get_files design_1.bd]
make_wrapper -files [get_files design_1.bd] -top -force
set wrapper [lindex [glob -nocomplain [file join $proj_dir ${proj_name}.gen sources_1 bd design_1 hdl design_1_wrapper.v]] 0]
if {$wrapper ne ""} {
    add_files -norecurse $wrapper
    set_property top design_1_wrapper [current_fileset]
}
update_compile_order -fileset sources_1
close_project

puts "Created FPU timer Vivado project: $new_xpr"
