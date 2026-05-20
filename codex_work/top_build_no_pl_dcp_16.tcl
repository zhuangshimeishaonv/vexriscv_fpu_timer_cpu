set project_root [file normalize "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"]
set project_dir  [file join $project_root "vivado_project"]
set xpr_path     [file join $project_dir "riaxiwithddr_fpu_timer.xpr"]
set output_dir   [file join $project_root "output"]

set_param general.maxThreads 16
file mkdir $output_dir

open_project $xpr_path

set generated_ip_dir [file normalize [file join $project_dir "riaxiwithddr_fpu_timer.gen" "sources_1" "bd" "design_1" "ip" "design_1_pl_channel_convolver_0_0"]]
set pl_dcp [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0.dcp"]
if {[file exists $pl_dcp]} {
    puts "Deleting stale PL DCP before top build: $pl_dcp"
    file delete -force $pl_dcp
} else {
    puts "PL DCP is absent, top synthesis must use RTL sources."
}

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
    exit 1
}

puts "Launching implementation through bitstream with 16 jobs..."
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
puts "impl_1 status: $impl_status"
if {![string match "*Complete*" $impl_status]} {
    puts "ERROR: impl_1 did not complete"
    exit 2
}

set bit_path [file join $project_dir "riaxiwithddr_fpu_timer.runs" "impl_1" "design_1_wrapper.bit"]
set out_bit  [file join $output_dir "riaxiwithddr_fpu_timer.bit"]
if {[file exists $bit_path]} {
    file copy -force $bit_path $out_bit
    puts "Copied bitstream to: $out_bit"
} else {
    puts "ERROR: bitstream not found at $bit_path"
    exit 3
}

close_project
puts "DONE"
