open_project {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
open_bd_design [get_files design_1.bd]
foreach space [get_bd_addr_spaces -quiet /VexRiscvAxi4_Wrapper_0/*] {
  puts "SPACE=$space"
  foreach seg [get_bd_addr_segs -quiet -of_objects $space] {
    puts "  SEG=$seg OFFSET=[get_property OFFSET $seg] RANGE=[get_property RANGE $seg]"
  }
}
close_project
