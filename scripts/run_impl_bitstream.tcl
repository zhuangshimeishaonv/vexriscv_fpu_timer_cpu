set xpr {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
set proj_dir [file normalize [file dirname $xpr]]
set vivado_ipi_dir {E:/vivado/Vivado/2022.2/scripts/ipintegrator}
set out_dir {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/output}
set rpt_dir {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/reports}
set build_jobs 16
file mkdir $out_dir
file mkdir $rpt_dir

set_param general.maxThreads $build_jobs

open_project $xpr
set_property ip_repo_paths [list \
  {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/ip_repo} \
] [current_project]
update_ip_catalog -rebuild

set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]

foreach run_name {design_1_pl_channel_convolver_0_0_synth_1 synth_1 impl_1} {
  if {[llength [get_runs -quiet $run_name]] > 0} {
    reset_run $run_name
  }
}

cd $vivado_ipi_dir
if {[llength [get_runs -quiet design_1_pl_channel_convolver_0_0_synth_1]] > 0} {
  launch_runs design_1_pl_channel_convolver_0_0_synth_1 -jobs $build_jobs
  wait_on_run design_1_pl_channel_convolver_0_0_synth_1
} else {
  puts "INFO: design_1_pl_channel_convolver_0_0_synth_1 not present; building PL convolver through top-level synth_1."
}

cd $proj_dir
reset_run synth_1
launch_runs synth_1 -jobs $build_jobs
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs $build_jobs
wait_on_run impl_1
cd $proj_dir

puts "IMPL_STATUS=[get_property STATUS [get_runs impl_1]]"
puts "IMPL_PROGRESS=[get_property PROGRESS [get_runs impl_1]]"

set bit_src [get_property DIRECTORY [get_runs impl_1]]/design_1_wrapper.bit
set bin_src [get_property DIRECTORY [get_runs impl_1]]/design_1_wrapper.bin
if {![file exists $bit_src]} {
  puts "ERROR: Expected bitstream not found at $bit_src"
  close_project
  exit 1
}

open_run impl_1
report_timing_summary -file "$rpt_dir/timing_summary_impl.rpt" -max_paths 20 -warn_on_violation
report_utilization -file "$rpt_dir/utilization_impl.rpt" -hierarchical
report_drc -file "$rpt_dir/drc_impl.rpt"
report_route_status -file "$rpt_dir/route_status_impl.rpt"

if {[file exists $bit_src]} {
  file copy -force $bit_src "$out_dir/riaxiwithddr_fpu_timer.bit"
}
if {[file exists $bin_src]} {
  file copy -force $bin_src "$out_dir/riaxiwithddr_fpu_timer.bin"
}
write_hw_platform -fixed -force -include_bit "$out_dir/riaxiwithddr_fpu_timer.xsa"
close_project
