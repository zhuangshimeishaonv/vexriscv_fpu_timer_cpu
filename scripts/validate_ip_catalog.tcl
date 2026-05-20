# Validate the packaged VexRiscv FPU Timer IP is visible to Vivado.
#
# Run:
#   E:/vivado/Vivado/2022.2/bin/vivado.bat -mode batch -source C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/scripts/validate_ip_catalog.tcl -nojournal -log C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/validate_ip.log

set script_dir [file normalize [file dirname [info script]]]
set root_dir   [file normalize [file join $script_dir ..]]
set proj_dir   [file normalize [file join $root_dir vivado_ip_validate]]
set repo_dir   [file normalize [file join $root_dir ip_repo]]
set part_name  "xc7z100ffg900-2"

create_project validate_vexriscv_fpu_timer_ip $proj_dir -part $part_name -force
set_property ip_repo_paths [list $repo_dir] [current_project]
update_ip_catalog -rebuild

set defs [get_ipdefs -all -quiet *VexRiscvAxi4FpuTimer_Wrapper*]
if {[llength $defs] == 0} {
    puts "ERROR: VexRiscvAxi4FpuTimer_Wrapper IP was not found in catalog."
    exit 1
}

puts "Found IP definition:"
puts $defs
close_project
