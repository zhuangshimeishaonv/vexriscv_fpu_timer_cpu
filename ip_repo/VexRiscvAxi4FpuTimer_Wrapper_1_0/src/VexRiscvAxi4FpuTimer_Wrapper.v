module VexRiscvAxi4FpuTimer_Wrapper (
    // ?????
    input  wire        clk,
    input wire      reset,
    input  wire        jtag_tms,
    input  wire        jtag_tdi,
    output wire        jtag_tdo,
    input  wire        jtag_tck,
    // ???? (iBusAxi) - ??
    output wire        iBusAxi_ar_valid,
    input  wire        iBusAxi_ar_ready,
    output wire [31:0] iBusAxi_ar_payload_addr,
    output wire [0:0]  iBusAxi_ar_payload_id,
    output wire [3:0]  iBusAxi_ar_payload_region,
    output wire [7:0]  iBusAxi_ar_payload_len,
    output wire [2:0]  iBusAxi_ar_payload_size,
    output wire [1:0]  iBusAxi_ar_payload_burst,
    output wire [0:0]  iBusAxi_ar_payload_lock,
    output wire [3:0]  iBusAxi_ar_payload_cache,
    output wire [3:0]  iBusAxi_ar_payload_qos,
    output wire [2:0]  iBusAxi_ar_payload_prot,
    input  wire        iBusAxi_r_valid,
    output wire        iBusAxi_r_ready,
    input  wire [31:0] iBusAxi_r_payload_data,
    input  wire [0:0]  iBusAxi_r_payload_id,
    input  wire [1:0]  iBusAxi_r_payload_resp,
    input  wire        iBusAxi_r_payload_last,

    // ???? (dBusAxi) - ??
    // ?????
    output wire        dBusAxi_aw_valid,
    input  wire        dBusAxi_aw_ready,
    output wire [31:0] dBusAxi_aw_payload_addr,
    output wire [0:0]  dBusAxi_aw_payload_id,
    output wire [3:0]  dBusAxi_aw_payload_region,
    output wire [7:0]  dBusAxi_aw_payload_len,
    output wire [2:0]  dBusAxi_aw_payload_size,
    output wire [1:0]  dBusAxi_aw_payload_burst,
    output wire [0:0]  dBusAxi_aw_payload_lock,
    output wire [3:0]  dBusAxi_aw_payload_cache,
    output wire [3:0]  dBusAxi_aw_payload_qos,
    output wire [2:0]  dBusAxi_aw_payload_prot,
    // ?????
    output wire        dBusAxi_w_valid,
    input  wire        dBusAxi_w_ready,
    output wire [31:0] dBusAxi_w_payload_data,
    output wire [3:0]  dBusAxi_w_payload_strb,
    output wire        dBusAxi_w_payload_last,
    // ?????
    input  wire        dBusAxi_b_valid,
    output wire        dBusAxi_b_ready,
    input  wire [0:0]  dBusAxi_b_payload_id,
    input  wire [1:0]  dBusAxi_b_payload_resp,
    // ?????
    output wire        dBusAxi_ar_valid,
    input  wire        dBusAxi_ar_ready,
    output wire [31:0] dBusAxi_ar_payload_addr,
    output wire [0:0]  dBusAxi_ar_payload_id,
    output wire [3:0]  dBusAxi_ar_payload_region,
    output wire [7:0]  dBusAxi_ar_payload_len,
    output wire [2:0]  dBusAxi_ar_payload_size,
    output wire [1:0]  dBusAxi_ar_payload_burst,
    output wire [0:0]  dBusAxi_ar_payload_lock,
    output wire [3:0]  dBusAxi_ar_payload_cache,
    output wire [3:0]  dBusAxi_ar_payload_qos,
    output wire [2:0]  dBusAxi_ar_payload_prot,
    // ?????
    input  wire        dBusAxi_r_valid,
    output wire        dBusAxi_r_ready,
    input  wire [31:0] dBusAxi_r_payload_data,
    input  wire [0:0]  dBusAxi_r_payload_id,
    input  wire [1:0]  dBusAxi_r_payload_resp,
    input  wire        dBusAxi_r_payload_last
);

    // ????????
    VexRiscvAxi4FpuTimer u_cpu_core (
        // --- ??????? ---
        .debug_resetOut         (),         // ???????
        .timerInterrupt         (1'b0),     // ?????
        .externalInterrupt      (1'b0),     // ?????
        .softwareInterrupt      (1'b0),     // ?????
        .jtag_tms              (jtag_tms),
        .jtag_tdi              (jtag_tdi),
        .jtag_tdo              (jtag_tdo),
        .jtag_tck              (jtag_tck),


        // --- ???? ---
        .clk                    (clk),
        .reset                  (~reset),

        // --- iBusAxi ?????? ---
        .iBusAxi_ar_valid       (iBusAxi_ar_valid),
        .iBusAxi_ar_ready       (iBusAxi_ar_ready),
        .iBusAxi_ar_payload_addr(iBusAxi_ar_payload_addr),
        .iBusAxi_ar_payload_id  (iBusAxi_ar_payload_id),
        .iBusAxi_ar_payload_region(iBusAxi_ar_payload_region),
        .iBusAxi_ar_payload_len (iBusAxi_ar_payload_len),
        .iBusAxi_ar_payload_size(iBusAxi_ar_payload_size),
        .iBusAxi_ar_payload_burst(iBusAxi_ar_payload_burst),
        .iBusAxi_ar_payload_lock(iBusAxi_ar_payload_lock),
        .iBusAxi_ar_payload_cache(iBusAxi_ar_payload_cache),
        .iBusAxi_ar_payload_qos (iBusAxi_ar_payload_qos),
        .iBusAxi_ar_payload_prot(iBusAxi_ar_payload_prot),
        .iBusAxi_r_valid        (iBusAxi_r_valid),
        .iBusAxi_r_ready        (iBusAxi_r_ready),
        .iBusAxi_r_payload_data (iBusAxi_r_payload_data),
        .iBusAxi_r_payload_id   (iBusAxi_r_payload_id),
        .iBusAxi_r_payload_resp (iBusAxi_r_payload_resp),
        .iBusAxi_r_payload_last (iBusAxi_r_payload_last),

        // --- dBusAxi ?????? ---
        .dBusAxi_aw_valid       (dBusAxi_aw_valid),
        .dBusAxi_aw_ready       (dBusAxi_aw_ready),
        .dBusAxi_aw_payload_addr(dBusAxi_aw_payload_addr),
        .dBusAxi_aw_payload_id  (dBusAxi_aw_payload_id),
        .dBusAxi_aw_payload_region(dBusAxi_aw_payload_region),
        .dBusAxi_aw_payload_len (dBusAxi_aw_payload_len),
        .dBusAxi_aw_payload_size(dBusAxi_aw_payload_size),
        .dBusAxi_aw_payload_burst(dBusAxi_aw_payload_burst),
        .dBusAxi_aw_payload_lock(dBusAxi_aw_payload_lock),
        .dBusAxi_aw_payload_cache(dBusAxi_aw_payload_cache),
        .dBusAxi_aw_payload_qos (dBusAxi_aw_payload_qos),
        .dBusAxi_aw_payload_prot(dBusAxi_aw_payload_prot),
        .dBusAxi_w_valid        (dBusAxi_w_valid),
        .dBusAxi_w_ready        (dBusAxi_w_ready),
        .dBusAxi_w_payload_data (dBusAxi_w_payload_data),
        .dBusAxi_w_payload_strb (dBusAxi_w_payload_strb),
        .dBusAxi_w_payload_last (dBusAxi_w_payload_last),
        .dBusAxi_b_valid        (dBusAxi_b_valid),
        .dBusAxi_b_ready        (dBusAxi_b_ready),
        .dBusAxi_b_payload_id   (dBusAxi_b_payload_id),
        .dBusAxi_b_payload_resp (dBusAxi_b_payload_resp),
        .dBusAxi_ar_valid       (dBusAxi_ar_valid),
        .dBusAxi_ar_ready       (dBusAxi_ar_ready),
        .dBusAxi_ar_payload_addr(dBusAxi_ar_payload_addr),
        .dBusAxi_ar_payload_id  (dBusAxi_ar_payload_id),
        .dBusAxi_ar_payload_region(dBusAxi_ar_payload_region),
        .dBusAxi_ar_payload_len (dBusAxi_ar_payload_len),
        .dBusAxi_ar_payload_size(dBusAxi_ar_payload_size),
        .dBusAxi_ar_payload_burst(dBusAxi_ar_payload_burst),
        .dBusAxi_ar_payload_lock(dBusAxi_ar_payload_lock),
        .dBusAxi_ar_payload_cache(dBusAxi_ar_payload_cache),
        .dBusAxi_ar_payload_qos (dBusAxi_ar_payload_qos),
        .dBusAxi_ar_payload_prot(dBusAxi_ar_payload_prot),
        .dBusAxi_r_valid        (dBusAxi_r_valid),
        .dBusAxi_r_ready        (dBusAxi_r_ready),
        .dBusAxi_r_payload_data (dBusAxi_r_payload_data),
        .dBusAxi_r_payload_id   (dBusAxi_r_payload_id),
        .dBusAxi_r_payload_resp (dBusAxi_r_payload_resp),
        .dBusAxi_r_payload_last (dBusAxi_r_payload_last)
    );

endmodule
