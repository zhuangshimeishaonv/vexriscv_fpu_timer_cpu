//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2022.2 (win64) Build 3671981 Fri Oct 14 05:00:03 MDT 2022
//Date        : Tue May 19 18:17:09 2026
//Host        : LAPTOP-I55VPB75 running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (DDR3_1_addr,
    DDR3_1_ba,
    DDR3_1_cas_n,
    DDR3_1_ck_n,
    DDR3_1_ck_p,
    DDR3_1_cke,
    DDR3_1_cs_n,
    DDR3_1_dm,
    DDR3_1_dq,
    DDR3_1_dqs_n,
    DDR3_1_dqs_p,
    DDR3_1_odt,
    DDR3_1_ras_n,
    DDR3_1_reset_n,
    DDR3_1_we_n,
    SYS_CLK_0_clk_n,
    SYS_CLK_0_clk_p,
    jtag_tck_0,
    jtag_tdi_0,
    jtag_tdo_0,
    jtag_tms_0,
    resetn_0,
    rx_0,
    tx_0);
  output [14:0]DDR3_1_addr;
  output [2:0]DDR3_1_ba;
  output DDR3_1_cas_n;
  output [0:0]DDR3_1_ck_n;
  output [0:0]DDR3_1_ck_p;
  output [0:0]DDR3_1_cke;
  output [0:0]DDR3_1_cs_n;
  output [3:0]DDR3_1_dm;
  inout [31:0]DDR3_1_dq;
  inout [3:0]DDR3_1_dqs_n;
  inout [3:0]DDR3_1_dqs_p;
  output [0:0]DDR3_1_odt;
  output DDR3_1_ras_n;
  output DDR3_1_reset_n;
  output DDR3_1_we_n;
  input SYS_CLK_0_clk_n;
  input SYS_CLK_0_clk_p;
  input jtag_tck_0;
  input jtag_tdi_0;
  output jtag_tdo_0;
  input jtag_tms_0;
  input resetn_0;
  input rx_0;
  output tx_0;

  wire [14:0]DDR3_1_addr;
  wire [2:0]DDR3_1_ba;
  wire DDR3_1_cas_n;
  wire [0:0]DDR3_1_ck_n;
  wire [0:0]DDR3_1_ck_p;
  wire [0:0]DDR3_1_cke;
  wire [0:0]DDR3_1_cs_n;
  wire [3:0]DDR3_1_dm;
  wire [31:0]DDR3_1_dq;
  wire [3:0]DDR3_1_dqs_n;
  wire [3:0]DDR3_1_dqs_p;
  wire [0:0]DDR3_1_odt;
  wire DDR3_1_ras_n;
  wire DDR3_1_reset_n;
  wire DDR3_1_we_n;
  wire SYS_CLK_0_clk_n;
  wire SYS_CLK_0_clk_p;
  wire jtag_tck_0;
  wire jtag_tdi_0;
  wire jtag_tdo_0;
  wire jtag_tms_0;
  wire resetn_0;
  wire rx_0;
  wire tx_0;

  design_1 design_1_i
       (.DDR3_1_addr(DDR3_1_addr),
        .DDR3_1_ba(DDR3_1_ba),
        .DDR3_1_cas_n(DDR3_1_cas_n),
        .DDR3_1_ck_n(DDR3_1_ck_n),
        .DDR3_1_ck_p(DDR3_1_ck_p),
        .DDR3_1_cke(DDR3_1_cke),
        .DDR3_1_cs_n(DDR3_1_cs_n),
        .DDR3_1_dm(DDR3_1_dm),
        .DDR3_1_dq(DDR3_1_dq),
        .DDR3_1_dqs_n(DDR3_1_dqs_n),
        .DDR3_1_dqs_p(DDR3_1_dqs_p),
        .DDR3_1_odt(DDR3_1_odt),
        .DDR3_1_ras_n(DDR3_1_ras_n),
        .DDR3_1_reset_n(DDR3_1_reset_n),
        .DDR3_1_we_n(DDR3_1_we_n),
        .SYS_CLK_0_clk_n(SYS_CLK_0_clk_n),
        .SYS_CLK_0_clk_p(SYS_CLK_0_clk_p),
        .jtag_tck_0(jtag_tck_0),
        .jtag_tdi_0(jtag_tdi_0),
        .jtag_tdo_0(jtag_tdo_0),
        .jtag_tms_0(jtag_tms_0),
        .resetn_0(resetn_0),
        .rx_0(rx_0),
        .tx_0(tx_0));
endmodule
