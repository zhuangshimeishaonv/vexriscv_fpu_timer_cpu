set proj "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr"
open_project $proj
open_bd_design [get_files design_1.bd]

puts "=== BD CELLS ==="
foreach c [lsort [get_bd_cells -hierarchical]] {
  puts $c
}

puts "=== PL CONVOLVER PINS ==="
foreach p [lsort [get_bd_pins -of_objects [get_bd_cells /pl_channel_convolver_0]]] {
  set net [get_bd_nets -of_objects $p]
  puts "$p -> $net"
}

puts "=== CONV DMA PINS ==="
if {[llength [get_bd_cells /axi_dma_conv -quiet]]} {
  foreach p [lsort [get_bd_pins -of_objects [get_bd_cells /axi_dma_conv]]] {
    set net [get_bd_nets -of_objects $p]
    puts "$p -> $net"
  }
}

puts "=== CLOCK/RESET NETS OF KEY CELLS ==="
foreach cell {/VexRiscvAxi4_Wrapper_0 /axi_uart16550_0 /axi_dma_0 /axi_dma_conv /pl_channel_convolver_0 /mig_7series_0} {
  if {[llength [get_bd_cells $cell -quiet]]} {
    puts "--- $cell"
    foreach p [get_bd_pins -of_objects [get_bd_cells $cell] -filter {TYPE == clk || TYPE == rst}] {
      puts "$p -> [get_bd_nets -of_objects $p]"
    }
  }
}

puts "=== ADDRESS SEGMENTS ==="
foreach seg [lsort [get_bd_addr_segs]] {
  set range [get_property range $seg]
  set offset [get_property offset $seg]
  puts "$seg OFFSET=$offset RANGE=$range"
}

puts "=== VALIDATE BD ==="
validate_bd_design
close_project
