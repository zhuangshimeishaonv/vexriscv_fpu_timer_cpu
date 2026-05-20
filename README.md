# VexRiscv FPU Timer CPU with PL Convolution

This is a VexRiscv-based SoC with FPU (Floating Point Unit), Timer, and Programmable Logic (PL) convolution acceleration, implemented on Xilinx FPGA using Vivado.

## Features

- **VexRiscv RISC-V CPU** with AXI4 bus interface
- **FPU (Floating Point Unit)** - hardware floating point support
- **Timer** - hardware timer peripheral
- **PL Convolution** - convolution accelerator in programmable logic
- **Integrated JTAG** - on-chip debug support
- **Vivado IP Packaging** - packaged as reusable Vivado IP

## Project Structure

- `vm_src/` - VexRiscv Scala source code and C test programs
- `hdl/` - Verilog HDL source files (top-level and wrapper)
- `scripts/` - Vivado Tcl scripts for project creation, synthesis, and implementation
- `ip_repo/` - Packaged Vivado IP repository
- `vivado_project/` - Vivado project files
- `output/` - Build output (bitstream, reports)
- `reports/` - Synthesis and implementation reports

## Key Files

- `hdl/VexRiscvAxi4FpuTimer.v` - Main VexRiscv Verilog with FPU and Timer
- `hdl/VexRiscvAxi4FpuTimer_Wrapper.v` - AXI4 wrapper for IP integration
- `vm_src/VexRiscvAxi4WithIntegratedJtagFpuTimer.scala` - Scala source for CPU generation
- `scripts/create_project_from_4_18_replace_cpu.tcl` - Vivado project creation script
- `scripts/run_impl_bitstream.tcl` - Implementation and bitstream generation

## Build

1. Open Vivado and source the project creation script
2. Run synthesis and implementation via the provided Tcl scripts
3. Generate bitstream and program the FPGA
