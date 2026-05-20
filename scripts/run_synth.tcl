set xpr {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
open_project $xpr
set_property ip_repo_paths [list \
  {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/ip_repo} \
  {C:/Users/minga/Desktop/ri} \
  {E:/mybishe/4.18/ip_repo} \
] [current_project]
update_ip_catalog -rebuild
reset_run synth_1
launch_runs synth_1 -jobs 1
wait_on_run synth_1
puts "SYNTH_STATUS=[get_property STATUS [get_runs synth_1]]"
puts "SYNTH_PROGRESS=[get_property PROGRESS [get_runs synth_1]]"
close_project
