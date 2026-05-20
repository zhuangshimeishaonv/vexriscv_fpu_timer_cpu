set project_root [file normalize "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"]
set project_dir  [file join $project_root "vivado_project"]
set xpr_path     [file join $project_dir "riaxiwithddr_fpu_timer.xpr"]
set ip_repo_dir  [file join $project_root "ip_repo"]
set output_dir   [file join $project_root "output"]

file mkdir $output_dir
set_param general.maxThreads 16

puts "Opening project: $xpr_path"
open_project $xpr_path

set_property ip_repo_paths [list $ip_repo_dir] [current_project]
update_ip_catalog -rebuild

set bd_files [get_files -quiet */design_1.bd]
if {[llength $bd_files] == 0} {
    set bd_files [get_files -quiet design_1.bd]
}
if {[llength $bd_files] == 0} {
    puts "ERROR: design_1.bd not found"
    exit 1
}
set bd_file [lindex $bd_files 0]
puts "Using BD: $bd_file"

open_bd_design $bd_file
validate_bd_design
save_bd_design

puts "Regenerating BD targets..."
generate_target all $bd_file

set ips_to_upgrade [get_ips -quiet]
if {[llength $ips_to_upgrade] > 0} {
    set upgrade_needed [get_ips -quiet -filter {IS_LOCKED == 1 || UPGRADE_VERSIONS != ""}]
    if {[llength $upgrade_needed] > 0} {
        puts "Upgrading locked/outdated IPs..."
        upgrade_ip $upgrade_needed
        generate_target all $bd_file
    }
}

set pl_run [get_runs -quiet design_1_pl_channel_convolver_0_0_synth_1]
if {[llength $pl_run] > 0} {
    puts "Resetting PL convolver OOC run..."
    reset_run $pl_run
}

foreach run_name {synth_1 impl_1} {
    set run_obj [get_runs -quiet $run_name]
    if {[llength $run_obj] > 0} {
        puts "Resetting $run_name..."
        reset_run $run_obj
    }
}

if {[llength $pl_run] > 0} {
    puts "Launching PL convolver OOC synthesis with 16 jobs..."
    launch_runs $pl_run -jobs 16
    wait_on_run $pl_run
    set pl_status [get_property STATUS $pl_run]
    puts "PL convolver OOC status: $pl_status"
    if {![string match "*Complete*" $pl_status]} {
        puts "ERROR: PL convolver OOC synthesis did not complete"
        exit 2
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
