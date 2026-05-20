# Package the generated VexRiscv AXI4 CPU with RV32IMF FPU + CSR timer.
#
# Run:
#   E:/vivado/Vivado/2022.2/bin/vivado.bat -mode batch -source C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/scripts/package_vexriscv_fpu_timer_ip.tcl -nojournal -log C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/package_ip.log
#
# Output:
#   C:/Users/minga/Desktop/vexriscv_fpu_timer_cpu/ip_repo/VexRiscvAxi4FpuTimer_Wrapper_1_0/component.xml

set script_dir [file normalize [file dirname [info script]]]
set root_dir   [file normalize [file join $script_dir ..]]
set hdl_dir    [file normalize [file join $root_dir hdl]]
set work_dir   [file normalize [file join $root_dir vivado_ip_pack]]
set ip_root    [file normalize [file join $root_dir ip_repo VexRiscvAxi4FpuTimer_Wrapper_1_0]]
set part_name  "xc7z100ffg900-2"

file mkdir $work_dir
file mkdir [file dirname $ip_root]

create_project vexriscv_fpu_timer_ip_pack $work_dir -part $part_name -force
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

add_files -fileset sources_1 [list \
    [file join $hdl_dir VexRiscvAxi4FpuTimer.v] \
    [file join $hdl_dir VexRiscvAxi4FpuTimer_Wrapper.v] \
]
set_property top VexRiscvAxi4FpuTimer_Wrapper [current_fileset]
update_compile_order -fileset sources_1

ipx::package_project \
    -root_dir $ip_root \
    -vendor xilinx.com \
    -library user \
    -taxonomy /UserIP \
    -import_files \
    -force

set core [ipx::current_core]
set_property name VexRiscvAxi4FpuTimer_Wrapper $core
set_property display_name {VexRiscv AXI4 FPU Timer Wrapper} $core
set_property description {VexRiscv AXI4 CPU with RV32IMF single-precision FPU and readable mcycle/minstret CSRs.} $core
set_property vendor_display_name {xilinx.com} $core
set_property company_url {https://github.com/SpinalHDL/VexRiscv} $core

ipx::infer_bus_interfaces xilinx.com:signal:clock_rtl:1.0 $core
ipx::infer_bus_interfaces xilinx.com:signal:reset_rtl:1.0 $core

ipx::update_checksums $core
ipx::save_core $core
close_project

puts "Packaged VexRiscvAxi4FpuTimer_Wrapper IP at: $ip_root"
