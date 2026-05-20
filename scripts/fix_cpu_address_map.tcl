set project_path {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr}
set out_dir {C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/reports}
file mkdir $out_dir

proc ensure_addr_seg {space_path slave_seg_path name offset range} {
  set space [get_bd_addr_spaces -quiet $space_path]
  set slave [get_bd_addr_segs -quiet $slave_seg_path]
  if {[llength $space] != 1} {
    error "Address space not found or ambiguous: $space_path"
  }
  if {[llength $slave] != 1} {
    error "Slave segment not found or ambiguous: $slave_seg_path"
  }

  set existing [get_bd_addr_segs -quiet "$space_path/$name"]
  if {[llength $existing] > 0} {
    set_property OFFSET $offset $existing
    set_property RANGE  $range  $existing
    puts "UPDATED $space_path/$name OFFSET=$offset RANGE=$range"
  } else {
    create_bd_addr_seg -range $range -offset $offset $space $slave $name
    puts "CREATED $space_path/$name OFFSET=$offset RANGE=$range"
  }
}

open_project $project_path
open_bd_design [get_files design_1.bd]

foreach cpu_space {/VexRiscvAxi4_Wrapper_0/dbus /VexRiscvAxi4_Wrapper_0/ibus} {
  ensure_addr_seg $cpu_space {/axi_bram_ctrl_0/S_AXI/Mem0}             {SEG_axi_bram_ctrl_0_Mem0}        0x00000000 0x00010000
  ensure_addr_seg $cpu_space {/axi_dma_0/S_AXI_LITE/Reg}              {SEG_axi_dma_0_Reg}               0x41E00000 0x00010000
  ensure_addr_seg $cpu_space {/axi_dma_conv/S_AXI_LITE/Reg}           {SEG_axi_dma_conv_Reg}            0x41F00000 0x00010000
  ensure_addr_seg $cpu_space {/pl_channel_convolver_0/s_axi/reg0}     {SEG_pl_channel_convolver_0_reg0} 0x42000000 0x00010000
  ensure_addr_seg $cpu_space {/mig_7series_0/memmap/memaddr}          {SEG_mig_7series_0_memaddr}       0x80000000 0x10000000
  ensure_addr_seg $cpu_space {/axi_uart16550_0/S_AXI/Reg}             {SEG_axi_uart16550_0_Reg}         0xF0010000 0x00010000
}

validate_bd_design
save_bd_design
generate_target synthesis [get_files design_1.bd] -force

set f [open "$out_dir/address_objects_after_fix.txt" w]
puts $f "ADDRESS SPACES"
foreach space [get_bd_addr_spaces -quiet -hierarchical] {
  puts $f "SPACE $space"
  foreach seg [get_bd_addr_segs -quiet -of_objects $space] {
    set off ""
    set rng ""
    set usage ""
    catch {set off [get_property OFFSET $seg]}
    catch {set rng [get_property RANGE $seg]}
    catch {set usage [get_property USAGE $seg]}
    puts $f "  SEG_OF_SPACE $seg OFFSET=$off RANGE=$rng USAGE=$usage"
  }
}
close $f

close_project
