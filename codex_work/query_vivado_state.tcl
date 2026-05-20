set project_root [file normalize "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu"]
set xpr_path [file join $project_root "vivado_project" "riaxiwithddr_fpu_timer.xpr"]
open_project $xpr_path
puts "RUNS:"
foreach r [get_runs] { puts "  $r :: [get_property STATUS $r]" }
puts "IPS:"
foreach ip [get_ips -quiet] { puts "  $ip :: VLNV=[get_property VLNV $ip] SYNTH=[get_property SYNTH_CHECKPOINT_MODE $ip]" }
puts "FILES matching convolver:"
foreach f [get_files -all -quiet *pl_channel_convolver*] { puts "  $f" }
close_project
