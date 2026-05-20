set proj "C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/vivado_project/riaxiwithddr_fpu_timer.xpr"
open_project $proj
open_bd_design [get_files design_1.bd]

puts "=== INTERFACE PINS: PL CONVOLVER ==="
foreach p [lsort [get_bd_intf_pins -of_objects [get_bd_cells /pl_channel_convolver_0]]] {
  puts "$p MODE=[get_property MODE $p] VLNV=[get_property VLNV $p] NET=[get_bd_intf_nets -of_objects $p]"
}

puts "=== INTERFACE PINS: AXI_DMA_CONV ==="
foreach p [lsort [get_bd_intf_pins -of_objects [get_bd_cells /axi_dma_conv]]] {
  puts "$p MODE=[get_property MODE $p] VLNV=[get_property VLNV $p] NET=[get_bd_intf_nets -of_objects $p]"
}

puts "=== INTERFACE NETS MATCHING CONV ==="
foreach n [lsort [get_bd_intf_nets -hierarchical]] {
  set pins [get_bd_intf_pins -of_objects $n]
  if {[regexp {conv|pl_channel|dma} $n] || [regexp {conv|pl_channel|dma} $pins]} {
    puts "$n -> $pins"
  }
}

puts "=== CONTROL BUS MASTERS/SLAVES ==="
foreach n [lsort [get_bd_intf_nets -hierarchical]] {
  set pins [get_bd_intf_pins -of_objects $n]
  if {[regexp {VexRiscv|axi_interconnect|pl_channel|uart|dma} $pins]} {
    puts "$n -> $pins"
  }
}

close_project
