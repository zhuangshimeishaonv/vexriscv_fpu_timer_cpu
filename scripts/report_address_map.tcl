set project_path {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
set out_dir {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/reports}
file mkdir $out_dir

open_project $project_path
open_bd_design [get_files design_1.bd]

set f [open "$out_dir/address_objects.txt" w]
puts $f "ADDRESS SPACES"
foreach space [get_bd_addr_spaces -quiet -hierarchical] {
  puts $f "SPACE $space"
  puts $f "  NAME=[get_property NAME $space]"
  foreach seg [get_bd_addr_segs -quiet -of_objects $space] {
    puts $f "  SEG_OF_SPACE $seg OFFSET=[get_property OFFSET $seg] RANGE=[get_property RANGE $seg] USAGE=[get_property USAGE $seg]"
  }
}

puts $f "\nADDRESS SEGS"
foreach seg [get_bd_addr_segs -quiet -hierarchical] {
  set off ""
  set rng ""
  set usage ""
  catch {set off [get_property OFFSET $seg]}
  catch {set rng [get_property RANGE $seg]}
  catch {set usage [get_property USAGE $seg]}
  puts $f "SEG $seg OFFSET=$off RANGE=$rng USAGE=$usage"
}
close $f

catch {report_bd_address -file "$out_dir/report_bd_address.txt"} err
if {$err ne ""} {
  set f [open "$out_dir/report_bd_address_error.txt" w]
  puts $f $err
  close $f
}

validate_bd_design
close_project
