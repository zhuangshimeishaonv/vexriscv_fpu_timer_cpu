set root "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"
set proj "$root/vivado_project/riaxiwithddr_fpu_timer.xpr"
set project_dir "$root/vivado_project"
set out_dir "$root/output"
set out_bit "$out_dir/riaxiwithddr_fpu_timer_old_pl.bit"
set out_main_bit "$out_dir/riaxiwithddr_fpu_timer.bit"

file mkdir $out_dir
open_project $proj
set_param general.maxThreads 16
update_ip_catalog -rebuild

set old_repo "$root/ip_repo/pl_channel_convolver_1_0/src/pl_channel_convolver_stream.v"
set old_ipshared "$project_dir/riaxiwithddr_fpu_timer.gen/sources_1/bd/design_1/ipshared/2c72/src/pl_channel_convolver_stream.v"
foreach f [list $old_repo $old_ipshared] {
    if {![file exists $f]} {
        error "Missing PL convolver source: $f"
    }
    set txt [read [open $f r]]
    if {[string first "x_rel_reg" $txt] < 0 || [string first "acc_real_mem" $txt] >= 0 || [string first "ST_CLEAR_ACC" $txt] >= 0} {
        error "PL convolver source does not look like old RTL: $f"
    }
}

set generated_ip_dir [file normalize [file join $project_dir "riaxiwithddr_fpu_timer.gen" "sources_1" "bd" "design_1" "ip" "design_1_pl_channel_convolver_0_0"]]
set stale_files [list \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0.dcp"] \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0_sim_netlist.v"] \
    [file join $generated_ip_dir "design_1_pl_channel_convolver_0_0_sim_netlist.vhdl"] \
]
foreach f $stale_files {
    if {[file exists $f]} {
        puts "Deleting stale generated file: $f"
        file delete -force $f
    }
}

set pl_run [get_runs -quiet design_1_pl_channel_convolver_0_0_synth_1]
if {[llength $pl_run]} {
    reset_run $pl_run
}
reset_run synth_1
reset_run impl_1

puts "Launching synth_1 with 16 jobs..."
launch_runs synth_1 -jobs 16
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
puts "synth_1 status: $synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "synth_1 did not complete"
}

puts "Launching impl_1/write_bitstream with 16 jobs..."
launch_runs impl_1 -to_step write_bitstream -jobs 16
wait_on_run impl_1
set impl_status [get_property STATUS [get_runs impl_1]]
puts "impl_1 status: $impl_status"
if {![string match "*Complete*" $impl_status]} {
    error "impl_1 did not complete"
}

set bit_path "$project_dir/riaxiwithddr_fpu_timer.runs/impl_1/design_1_wrapper.bit"
if {![file exists $bit_path]} {
    error "Bitstream not found: $bit_path"
}
file copy -force $bit_path $out_bit
file copy -force $bit_path $out_main_bit
puts "Copied old-PL bitstream to: $out_bit"
puts "Copied old-PL bitstream to default output: $out_main_bit"

close_project
