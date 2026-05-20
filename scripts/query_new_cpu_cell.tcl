set xpr {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
open_project $xpr
set_property ip_repo_paths [list \
  {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/ip_repo} \
  {C:/Users/minga/Desktop/ri} \
  {E:/mybishe/4.18/ip_repo} \
] [current_project]
update_ip_catalog -rebuild
open_bd_design [get_files design_1.bd]
puts "Cells:"
puts [get_bd_cells -quiet *Vex*]
foreach c [get_bd_cells -quiet *Vex*] {
  puts "CELL=$c"
  puts "INTF_PINS=[get_bd_intf_pins -quiet -of_objects $c]"
  puts "PINS=[get_bd_pins -quiet -of_objects $c]"
}
close_project
