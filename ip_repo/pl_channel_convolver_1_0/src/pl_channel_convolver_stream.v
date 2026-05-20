`timescale 1ns / 1ps
/*
 * Fixed-point segmented multipath convolver.
 *
 * Input stream packet after start:
 *   1) x_len words: signed Q1.15 x in s_axis_tdata[15:0]
 *   2) num_snapshots*num_paths words: {h_imag_q15, h_real_q15}
 *   3) num_snapshots*num_paths words: delay_samples in [15:0], 16'hffff invalid
 *
 * Output stream packet:
 *   for n = 0..y_len-1:
 *     word 0: y_real_q15 as signed int32
 *     word 1: y_imag_q15 as signed int32
 *   m_axis_tlast is asserted on the final imag word.
 *
 * The arithmetic matches pl_conv_reference_segmented_q15(): Q15*Q15 is
 * accumulated in Q30, then rounded back to Q15 in signed 32-bit output.
 */
module pl_channel_convolver_stream #(
    parameter integer MAX_X_LEN = 2048,
    parameter integer MAX_Y_LEN = 6144,
    parameter integer MAX_SNAPSHOTS = 4,
    parameter integer MAX_PATHS = 80,
    parameter integer ACC_WIDTH = 48
) (
    input  wire         aclk,
    input  wire         aresetn,

    input  wire         start,
    input  wire [15:0]  cfg_x_len,
    input  wire [15:0]  cfg_y_len,
    input  wire [15:0]  cfg_samples_per_snapshot,
    input  wire [7:0]   cfg_num_snapshots,
    input  wire [7:0]   cfg_num_paths,
    output reg          busy,
    output reg          done,
    output reg          error,

    input  wire [31:0]  s_axis_tdata,
    input  wire         s_axis_tvalid,
    output reg          s_axis_tready,

    output reg  [31:0]  m_axis_tdata,
    output reg          m_axis_tvalid,
    input  wire         m_axis_tready,
    output reg          m_axis_tlast
);

    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_LOAD_X    = 4'd1,
        ST_LOAD_H    = 4'd2,
        ST_LOAD_D    = 4'd3,
        ST_PREP      = 4'd4,
        ST_READ_PATH = 4'd5,
        ST_READ_XADR = 4'd6,
        ST_SET_XADR  = 4'd7,
        ST_READ_X    = 4'd8,
        ST_MUL       = 4'd9,
        ST_ACC       = 4'd10,
        ST_ADVANCE   = 4'd11,
        ST_ROUND     = 4'd12,
        ST_SEND_REAL = 4'd13,
        ST_SEND_IMAG = 4'd14,
        ST_DONE      = 4'd15;

    reg [3:0] state;

    (* ram_style = "block" *) reg signed [15:0] x_mem [0:MAX_X_LEN-1];
    (* ram_style = "block" *) reg signed [15:0] h_real_mem [0:MAX_SNAPSHOTS*MAX_PATHS-1];
    (* ram_style = "block" *) reg signed [15:0] h_imag_mem [0:MAX_SNAPSHOTS*MAX_PATHS-1];
    (* ram_style = "block" *) reg [15:0] delay_mem [0:MAX_SNAPSHOTS*MAX_PATHS-1];

    reg [15:0] load_count;
    reg [15:0] out_n;
    reg [7:0] snap_idx;
    reg [7:0] path_idx;
    reg signed [ACC_WIDTH-1:0] acc_real;
    reg signed [ACC_WIDTH-1:0] acc_imag;
    reg signed [31:0] y_real_hold;
    reg signed [31:0] y_imag_hold;
    reg signed [15:0] mac_x;
    reg signed [15:0] mac_h_real;
    reg signed [15:0] mac_h_imag;
    reg mac_valid;
    reg [15:0] x_lookup_addr_reg;
    reg [15:0] path_linear_reg;
    reg [15:0] delay_current_reg;
    reg signed [31:0] x_rel_reg;
    reg [31:0] seg_start_reg;
    reg signed [31:0] product_real_reg;
    reg signed [31:0] product_imag_reg;

    wire [15:0] total_paths = cfg_num_snapshots * cfg_num_paths;
    wire [15:0] path_linear = snap_idx * cfg_num_paths + path_idx;
    wire [31:0] seg_start = snap_idx * cfg_samples_per_snapshot;
    wire signed [31:0] product_real = mac_x * mac_h_real;
    wire signed [31:0] product_imag = mac_x * mac_h_imag;
    wire signed [ACC_WIDTH-1:0] add_real =
        {{(ACC_WIDTH-32){product_real_reg[31]}}, product_real_reg};
    wire signed [ACC_WIDTH-1:0] add_imag =
        {{(ACC_WIDTH-32){product_imag_reg[31]}}, product_imag_reg};

    function signed [31:0] round_q30_to_q15;
        input signed [ACC_WIDTH-1:0] value;
        reg signed [ACC_WIDTH-1:0] mag;
        begin
            if (value >= 0) begin
                round_q30_to_q15 = (value + ({{(ACC_WIDTH-16){1'b0}}, 16'd16384})) >>> 15;
            end else begin
                mag = -value;
                round_q30_to_q15 = -((mag + ({{(ACC_WIDTH-16){1'b0}}, 16'd16384})) >>> 15);
            end
        end
    endfunction

    always @(posedge aclk) begin
        if (!aresetn) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
            s_axis_tready <= 1'b0;
            m_axis_tdata <= 32'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast <= 1'b0;
            load_count <= 16'd0;
            out_n <= 16'd0;
            snap_idx <= 8'd0;
            path_idx <= 8'd0;
            acc_real <= {ACC_WIDTH{1'b0}};
            acc_imag <= {ACC_WIDTH{1'b0}};
            y_real_hold <= 32'sd0;
            y_imag_hold <= 32'sd0;
            mac_x <= 16'sd0;
            mac_h_real <= 16'sd0;
            mac_h_imag <= 16'sd0;
            mac_valid <= 1'b0;
            x_lookup_addr_reg <= 16'd0;
            path_linear_reg <= 16'd0;
            delay_current_reg <= 16'hffff;
            x_rel_reg <= 32'sd0;
            seg_start_reg <= 32'd0;
            product_real_reg <= 32'sd0;
            product_imag_reg <= 32'sd0;
        end else begin
            done <= 1'b0;

            case (state)
            ST_IDLE: begin
                busy <= 1'b0;
                s_axis_tready <= 1'b0;
                m_axis_tvalid <= 1'b0;
                m_axis_tlast <= 1'b0;
                if (start) begin
                    busy <= 1'b1;
                    error <= (cfg_x_len > MAX_X_LEN) ||
                             (cfg_y_len > MAX_Y_LEN) ||
                             (cfg_num_snapshots > MAX_SNAPSHOTS) ||
                             (cfg_num_paths > MAX_PATHS) ||
                             (cfg_samples_per_snapshot == 0);
                    load_count <= 16'd0;
                    s_axis_tready <= 1'b1;
                    state <= ((cfg_x_len > MAX_X_LEN) ||
                              (cfg_y_len > MAX_Y_LEN) ||
                              (cfg_num_snapshots > MAX_SNAPSHOTS) ||
                              (cfg_num_paths > MAX_PATHS) ||
                              (cfg_samples_per_snapshot == 0)) ? ST_DONE : ST_LOAD_X;
                end
            end

            ST_LOAD_X: begin
                s_axis_tready <= 1'b1;
                if (s_axis_tvalid && s_axis_tready) begin
                    x_mem[load_count] <= s_axis_tdata[15:0];
                    load_count <= load_count + 16'd1;
                    if (load_count + 16'd1 == cfg_x_len) begin
                        load_count <= 16'd0;
                        state <= ST_LOAD_H;
                    end
                end
            end

            ST_LOAD_H: begin
                s_axis_tready <= 1'b1;
                if (s_axis_tvalid && s_axis_tready) begin
                    h_real_mem[load_count] <= s_axis_tdata[15:0];
                    h_imag_mem[load_count] <= s_axis_tdata[31:16];
                    load_count <= load_count + 16'd1;
                    if (load_count + 16'd1 == total_paths) begin
                        load_count <= 16'd0;
                        state <= ST_LOAD_D;
                    end
                end
            end

            ST_LOAD_D: begin
                s_axis_tready <= 1'b1;
                if (s_axis_tvalid && s_axis_tready) begin
                    delay_mem[load_count] <= s_axis_tdata[15:0];
                    load_count <= load_count + 16'd1;
                    if (load_count + 16'd1 == total_paths) begin
                        s_axis_tready <= 1'b0;
                        out_n <= 16'd0;
                        snap_idx <= 8'd0;
                        path_idx <= 8'd0;
                        acc_real <= {ACC_WIDTH{1'b0}};
                        acc_imag <= {ACC_WIDTH{1'b0}};
                        state <= ST_PREP;
                    end
                end
            end

            ST_PREP: begin
                path_linear_reg <= path_linear;
                seg_start_reg <= seg_start;
                state <= ST_READ_PATH;
            end

            ST_READ_PATH: begin
                mac_h_real <= h_real_mem[path_linear_reg];
                mac_h_imag <= h_imag_mem[path_linear_reg];
                delay_current_reg <= delay_mem[path_linear_reg];
                state <= ST_READ_XADR;
            end

            ST_READ_XADR: begin
                x_rel_reg <= $signed({1'b0, out_n}) -
                             $signed({1'b0, seg_start_reg[15:0]}) -
                             $signed({1'b0, delay_current_reg});
                state <= ST_SET_XADR;
            end

            ST_SET_XADR: begin
                if (delay_current_reg != 16'hffff &&
                    x_rel_reg >= 0 &&
                    x_rel_reg < $signed({1'b0, cfg_samples_per_snapshot}) &&
                    (seg_start_reg[15:0] + x_rel_reg[15:0]) < cfg_x_len &&
                    (seg_start_reg[15:0] + x_rel_reg[15:0]) < MAX_X_LEN) begin
                    x_lookup_addr_reg <= seg_start_reg[15:0] + x_rel_reg[15:0];
                    mac_valid <= 1'b1;
                end else begin
                    x_lookup_addr_reg <= 16'd0;
                    mac_valid <= 1'b0;
                end
                state <= ST_READ_X;
            end

            ST_READ_X: begin
                mac_x <= x_mem[x_lookup_addr_reg];
                state <= ST_MUL;
            end

            ST_MUL: begin
                product_real_reg <= mac_valid ? product_real : 32'sd0;
                product_imag_reg <= mac_valid ? product_imag : 32'sd0;
                state <= ST_ACC;
            end

            ST_ACC: begin
                acc_real <= acc_real + add_real;
                acc_imag <= acc_imag + add_imag;
                state <= ST_ADVANCE;
            end

            ST_ADVANCE: begin
                if (path_idx + 8'd1 < cfg_num_paths) begin
                    path_idx <= path_idx + 8'd1;
                    state <= ST_PREP;
                end else begin
                    path_idx <= 8'd0;
                    if (snap_idx + 8'd1 < cfg_num_snapshots) begin
                        snap_idx <= snap_idx + 8'd1;
                        state <= ST_PREP;
                    end else begin
                        snap_idx <= 8'd0;
                        state <= ST_ROUND;
                    end
                end
            end

            ST_ROUND: begin
                y_real_hold <= round_q30_to_q15(acc_real);
                y_imag_hold <= round_q30_to_q15(acc_imag);
                state <= ST_SEND_REAL;
            end

            ST_SEND_REAL: begin
                m_axis_tdata <= y_real_hold;
                m_axis_tvalid <= 1'b1;
                m_axis_tlast <= 1'b0;
                if (m_axis_tvalid && m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    state <= ST_SEND_IMAG;
                end
            end

            ST_SEND_IMAG: begin
                m_axis_tdata <= y_imag_hold;
                m_axis_tvalid <= 1'b1;
                m_axis_tlast <= (out_n + 16'd1 == cfg_y_len);
                if (m_axis_tvalid && m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast <= 1'b0;
                    if (out_n + 16'd1 == cfg_y_len) begin
                        state <= ST_DONE;
                    end else begin
                        out_n <= out_n + 16'd1;
                        path_idx <= 8'd0;
                        snap_idx <= 8'd0;
                        acc_real <= {ACC_WIDTH{1'b0}};
                        acc_imag <= {ACC_WIDTH{1'b0}};
                        state <= ST_PREP;
                    end
                end
            end

            ST_DONE: begin
                busy <= 1'b0;
                done <= 1'b1;
                s_axis_tready <= 1'b0;
                m_axis_tvalid <= 1'b0;
                m_axis_tlast <= 1'b0;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
