set project_root [file normalize "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"]
set project_dir  [file join $project_root "vivado_project"]
set xpr_path     [file join $project_dir "riaxiwithddr_fpu_timer.xpr"]
set ip_repo_dir  [file join $project_root "ip_repo"]
set output_dir   [file join $project_root "output"]

set_param general.maxThreads 16
file mkdir $output_dir

open_project $xpr_path
set_property ip_repo_paths [list $ip_repo_dir] [current_project]
update_ip_catalog -rebuild

set bd_file [lindex [get_files -quiet */design_1.bd] 0]
if {$bd_file eq ""} {
    puts "ERROR: design_1.bd not found"
    exit 1
}
open_bd_design $bd_file

set pl_xci [lindex [get_files -quiet *design_1_pl_channel_convolver_0_0.xci] 0]
set pl_ip  [lindex [get_ips -quiet design_1_pl_channel_convolver_0_0] 0]
if {$pl_xci eq "" || $pl_ip eq ""} {
    puts "ERROR: PL convolver IP/XCI not found"
    exit 2
}

puts "PL IP:  $pl_ip"
puts "PL XCI: $pl_xci"

set generated_ip_dir [file normalize [file join $project_dir "riaxiwithddr_fpu_timer.gen" "sources_1" "bd" "design_1" "ip" "design_1_pl_channel_convolver_0_0"]]
set stale_files [list \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0.dcp"] \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0_sim_netlist.v"] \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0_sim_netlist.vhdl"] \
    [file join $generated_ip_dir "synth" "design_1_pl_channel_convolver_0_0.v"] \
    [file join $generated_ip_dir "sim" "design_1_pl_channel_convolver_0_0.v"] \
]
foreach f $stale_files {
    if {[file exists $f]} {
        puts "Deleting stale generated file: $f"
        file delete -force $f
    }
}

puts "Resetting and regenerating PL IP targets..."
reset_target all $pl_xci
generate_target all -force $pl_xci

puts "Synthesizing PL IP checkpoint from current RTL..."
synth_ip -force $pl_ip

puts "Regenerating BD wrapper targets..."
generate_target all -force $bd_file
validate_bd_design
save_bd_design

foreach run_name {synth_1 impl_1} {
    set run_obj [get_runs -quiet $run_name]
    if {[llength $run_obj] > 0} {
        puts "Resetting $run_name..."
        reset_run $run_obj
    }
}

puts "Launching top synthesis with 16 jobs..."
launch_runs synth_1 -jobs 16
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
puts "synth_1 status: $synth_status"
if {![string match "*Complete*" $synth_status]} {
    puts "ERROR: synth_1 did not complete"
    exit 3
}

puts "Launching implementation through bitstream with 16 jobs..."
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
puts "impl_1 status: $impl_status"
if {![string match "*Complete*" $impl_status]} {
    puts "ERROR: impl_1 did not complete"
    exit 4
}

set bit_path [file join $project_dir "riaxiwithddr_fpu_timer.runs" "impl_1" "design_1_wrapper.bit"]
set out_bit  [file join $output_dir "riaxiwithddr_fpu_timer.bit"]
if {[file exists $bit_path]} {
    file copy -force $bit_path $out_bit
    puts "Copied bitstream to: $out_bit"
} else {
    puts "ERROR: bitstream not found at $bit_path"
    exit 5
}

close_project
puts "DONE"
