`timescale 1ns / 1ps

module pl_channel_convolver_axi_lite #(
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer MAX_X_LEN = 2048,
    parameter integer MAX_Y_LEN = 6144,
    parameter integer MAX_SNAPSHOTS = 4,
    parameter integer MAX_PATHS = 80
) (
    input  wire                             s_axi_aclk,
    input  wire                             s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire                             s_axi_awvalid,
    output reg                              s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                             s_axi_wvalid,
    output reg                              s_axi_wready,
    output reg  [1:0]                       s_axi_bresp,
    output reg                              s_axi_bvalid,
    input  wire                             s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire                             s_axi_arvalid,
    output reg                              s_axi_arready,
    output reg  [C_S_AXI_DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]                       s_axi_rresp,
    output reg                              s_axi_rvalid,
    input  wire                             s_axi_rready,

    input  wire [31:0]                      s_axis_tdata,
    input  wire                             s_axis_tvalid,
    output wire                             s_axis_tready,

    output wire [31:0]                      m_axis_tdata,
    output wire                             m_axis_tvalid,
    input  wire                             m_axis_tready,
    output wire                             m_axis_tlast
);

    localparam [5:0]
        REG_CTRL                 = 6'h00,
        REG_X_LEN                = 6'h10,
        REG_Y_LEN                = 6'h14,
        REG_SAMPLES_PER_SNAPSHOT = 6'h18,
        REG_NUM_SNAPSHOTS        = 6'h1c,
        REG_NUM_PATHS            = 6'h20;

    reg [15:0] cfg_x_len;
    reg [15:0] cfg_y_len;
    reg [15:0] cfg_samples_per_snapshot;
    reg [7:0] cfg_num_snapshots;
    reg [7:0] cfg_num_paths;
    reg start_pulse;
    wire core_busy;
    wire core_done;
    wire core_error;
    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_hold;
    reg [C_S_AXI_DATA_WIDTH-1:0] wdata_hold;
    reg [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb_hold;
    reg aw_holding;
    reg w_holding;

    pl_channel_convolver_stream #(
        .MAX_X_LEN(MAX_X_LEN),
        .MAX_Y_LEN(MAX_Y_LEN),
        .MAX_SNAPSHOTS(MAX_SNAPSHOTS),
        .MAX_PATHS(MAX_PATHS)
    ) core_i (
        .aclk(s_axi_aclk),
        .aresetn(s_axi_aresetn),
        .start(start_pulse),
        .cfg_x_len(cfg_x_len),
        .cfg_y_len(cfg_y_len),
        .cfg_samples_per_snapshot(cfg_samples_per_snapshot),
        .cfg_num_snapshots(cfg_num_snapshots),
        .cfg_num_paths(cfg_num_paths),
        .busy(core_busy),
        .done(core_done),
        .error(core_error),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast)
    );

    wire aw_accept = s_axi_awvalid && !aw_holding && !s_axi_bvalid;
    wire w_accept = s_axi_wvalid && !w_holding && !s_axi_bvalid;
    wire write_fire = (aw_holding || aw_accept) && (w_holding || w_accept) && !s_axi_bvalid;
    wire [C_S_AXI_ADDR_WIDTH-1:0] write_addr = aw_accept ? s_axi_awaddr : awaddr_hold;
    wire [C_S_AXI_DATA_WIDTH-1:0] write_data = w_accept ? s_axi_wdata : wdata_hold;
    wire [(C_S_AXI_DATA_WIDTH/8)-1:0] write_strb = w_accept ? s_axi_wstrb : wstrb_hold;
    wire read_fire = s_axi_arvalid && !s_axi_rvalid;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            cfg_x_len <= 16'd0;
            cfg_y_len <= 16'd0;
            cfg_samples_per_snapshot <= 16'd0;
            cfg_num_snapshots <= 8'd0;
            cfg_num_paths <= 8'd0;
            start_pulse <= 1'b0;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bresp <= 2'b00;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rdata <= 32'd0;
            s_axi_rresp <= 2'b00;
            s_axi_rvalid <= 1'b0;
            awaddr_hold <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            wdata_hold <= {C_S_AXI_DATA_WIDTH{1'b0}};
            wstrb_hold <= {(C_S_AXI_DATA_WIDTH/8){1'b0}};
            aw_holding <= 1'b0;
            w_holding <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            s_axi_awready <= aw_accept;
            s_axi_wready <= w_accept;
            s_axi_arready <= read_fire;

            if (aw_accept) begin
                awaddr_hold <= s_axi_awaddr;
                aw_holding <= 1'b1;
            end
            if (w_accept) begin
                wdata_hold <= s_axi_wdata;
                wstrb_hold <= s_axi_wstrb;
                w_holding <= 1'b1;
            end

            if (write_fire) begin
                case (write_addr[5:0])
                REG_CTRL: begin
                    if (write_strb[0] && write_data[0]) begin
                        start_pulse <= 1'b1;
                    end
                end
                REG_X_LEN: begin
                    if (write_strb[0]) cfg_x_len[7:0] <= write_data[7:0];
                    if (write_strb[1]) cfg_x_len[15:8] <= write_data[15:8];
                end
                REG_Y_LEN: begin
                    if (write_strb[0]) cfg_y_len[7:0] <= write_data[7:0];
                    if (write_strb[1]) cfg_y_len[15:8] <= write_data[15:8];
                end
                REG_SAMPLES_PER_SNAPSHOT: begin
                    if (write_strb[0]) cfg_samples_per_snapshot[7:0] <= write_data[7:0];
                    if (write_strb[1]) cfg_samples_per_snapshot[15:8] <= write_data[15:8];
                end
                REG_NUM_SNAPSHOTS: begin
                    if (write_strb[0]) cfg_num_snapshots <= write_data[7:0];
                end
                REG_NUM_PATHS: begin
                    if (write_strb[0]) cfg_num_paths <= write_data[7:0];
                end
                default: begin end
                endcase
                aw_holding <= 1'b0;
                w_holding <= 1'b0;
                s_axi_bresp <= 2'b00;
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (read_fire) begin
                case (s_axi_araddr[5:0])
                REG_CTRL: s_axi_rdata <= {28'd0, core_error, core_busy, core_done, 1'b0};
                REG_X_LEN: s_axi_rdata <= {16'd0, cfg_x_len};
                REG_Y_LEN: s_axi_rdata <= {16'd0, cfg_y_len};
                REG_SAMPLES_PER_SNAPSHOT: s_axi_rdata <= {16'd0, cfg_samples_per_snapshot};
                REG_NUM_SNAPSHOTS: s_axi_rdata <= {24'd0, cfg_num_snapshots};
                REG_NUM_PATHS: s_axi_rdata <= {24'd0, cfg_num_paths};
                default: s_axi_rdata <= 32'd0;
                endcase
                s_axi_rresp <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
