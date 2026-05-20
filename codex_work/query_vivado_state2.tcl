set project_root [file normalize "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"]
set xpr_path [file join $project_root "vivado_project" "riaxiwithddr_fpu_timer.xpr"]
open_project $xpr_path
puts "IPS:"
foreach ip [get_ips -quiet] {
  puts "  IP=$ip"
  foreach prop {IPDEF VLNV CORE_CONTAINER SYNTH_CHECKPOINT_MODE IP_FILE IS_LOCKED UPGRADE_VERSIONS} {
    set val "<NA>"
    catch {set val [get_property $prop $ip]}
    puts "    $prop=$val"
  }
}
puts "FILES matching convolver:"
foreach f [get_files -all -quiet *pl_channel_convolver*] { puts "  $f" }
close_project
