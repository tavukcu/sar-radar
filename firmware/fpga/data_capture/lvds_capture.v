//============================================================================
// lvds_capture.v
// AWR2243 LVDS IF Data Capture Module
//
// The AWR2243 outputs digitized IF (intermediate frequency) data over
// 4 LVDS data lanes (one per RX antenna channel), plus a frame clock
// and a bit clock. This module deserializes the high-speed LVDS streams
// into parallel 12-bit ADC samples.
//
// Architecture:
//   - IBUFDS for differential-to-single conversion
//   - IDELAYE2 for per-lane delay tuning (optional, for bitslip)
//   - ISERDESE2 for high-speed deserialization (Artix-7 specific)
//   - Bitslip alignment using known frame sync pattern
//   - Async FIFO for clock domain crossing (LVDS clk -> processing clk)
//
// AWR2243 LVDS output specifications:
//   - Data rate: up to 600 Mbps per lane
//   - Encoding: 12-bit ADC samples, MSB first
//   - Frame sync: unique pattern at start of each chirp
//   - Lanes: 4 data + 1 frame clock + 1 bit clock
//
// Target FPGA: Xilinx Artix-7 XC7A100T-1CPG236C
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module lvds_capture (
    //------------------------------------------------------------------
    // System Clocks
    //------------------------------------------------------------------
    input  wire        clk_sys,          // 100 MHz system clock
    input  wire        clk_proc,         // 200 MHz processing clock
    input  wire        rst,              // Synchronous reset (clk_sys domain)

    //------------------------------------------------------------------
    // LVDS Physical Interface (from AWR2243)
    //------------------------------------------------------------------
    input  wire        lvds_clk_p,       // LVDS bit clock positive
    input  wire        lvds_clk_n,       // LVDS bit clock negative
    input  wire        lvds_frame_p,     // LVDS frame clock positive
    input  wire        lvds_frame_n,     // LVDS frame clock negative
    input  wire [3:0]  lvds_data_p,      // LVDS data lanes positive
    input  wire [3:0]  lvds_data_n,      // LVDS data lanes negative

    //------------------------------------------------------------------
    // Control
    //------------------------------------------------------------------
    input  wire        enable,           // Enable capture
    output reg         aligned,          // Bitslip alignment achieved

    //------------------------------------------------------------------
    // Data Output (clk_proc domain)
    //------------------------------------------------------------------
    output reg         data_valid,       // Output sample valid
    output reg  [11:0] ch0_data,         // RX channel 0 ADC sample
    output reg  [11:0] ch1_data,         // RX channel 1 ADC sample
    output reg  [11:0] ch2_data,         // RX channel 2 ADC sample
    output reg  [11:0] ch3_data,         // RX channel 3 ADC sample
    output reg         chirp_start,      // Chirp boundary detected
    output reg         frame_sync,       // Frame sync pattern detected
    output reg         overflow          // FIFO overflow indicator
);

    //======================================================================
    // Parameters
    //======================================================================
    localparam SERDES_FACTOR = 6;         // 6:1 deserialization (2 cycles = 12 bits)
    localparam FRAME_SYNC_PATTERN = 12'hFFF; // Known sync pattern (all ones)
    localparam ALIGN_THRESHOLD = 8;       // Consecutive good frames to declare aligned

    //======================================================================
    // LVDS Input Buffers
    //======================================================================
    wire       lvds_clk_ibuf;            // Single-ended bit clock
    wire       lvds_frame_ibuf;          // Single-ended frame clock
    wire [3:0] lvds_data_ibuf;           // Single-ended data lanes

    // Bit clock differential buffer -> BUFIO for high-speed clocking
    wire lvds_clk_bufio;                 // Fast clock for ISERDES
    wire lvds_clk_bufr;                  // Divided clock for parallel domain

    IBUFDS #(
        .DIFF_TERM  ("TRUE"),
        .IOSTANDARD ("LVDS_25")
    ) u_ibufds_clk (
        .I  (lvds_clk_p),
        .IB (lvds_clk_n),
        .O  (lvds_clk_ibuf)
    );

    // BUFIO: high-speed clock buffer for ISERDES (no routing, clock region only)
    BUFIO u_bufio_clk (
        .I (lvds_clk_ibuf),
        .O (lvds_clk_bufio)
    );

    // BUFR: divided clock for parallel data domain (bit_clk / SERDES_FACTOR)
    BUFR #(
        .BUFR_DIVIDE ("6"),
        .SIM_DEVICE  ("7SERIES")
    ) u_bufr_clk (
        .I   (lvds_clk_ibuf),
        .O   (lvds_clk_bufr),
        .CE  (1'b1),
        .CLR (rst)
    );

    // Frame clock differential buffer
    IBUFDS #(
        .DIFF_TERM  ("TRUE"),
        .IOSTANDARD ("LVDS_25")
    ) u_ibufds_frame (
        .I  (lvds_frame_p),
        .IB (lvds_frame_n),
        .O  (lvds_frame_ibuf)
    );

    // Data lane differential buffers
    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : gen_ibufds_data
            IBUFDS #(
                .DIFF_TERM  ("TRUE"),
                .IOSTANDARD ("LVDS_25")
            ) u_ibufds_data (
                .I  (lvds_data_p[g]),
                .IB (lvds_data_n[g]),
                .O  (lvds_data_ibuf[g])
            );
        end
    endgenerate

    //======================================================================
    // ISERDESE2 Deserialization (one per data lane + frame)
    // Using 6:1 DDR mode: each ISERDES produces 6 bits per parallel clock
    // Two parallel clock cycles yield 12 bits (one ADC sample)
    //======================================================================
    wire [5:0] iserdes_data [0:3];       // Deserialized data per lane
    wire [5:0] iserdes_frame;            // Deserialized frame clock
    reg  [3:0] bitslip;                  // Bitslip control per lane
    reg        bitslip_frame;            // Bitslip for frame lane

    // Frame clock ISERDES
    ISERDESE2 #(
        .DATA_RATE       ("DDR"),
        .DATA_WIDTH      (6),
        .INTERFACE_TYPE  ("NETWORKING"),
        .NUM_CE          (1),
        .SERDES_MODE     ("MASTER"),
        .IOBDELAY        ("NONE")
    ) u_iserdes_frame (
        .Q1       (iserdes_frame[0]),
        .Q2       (iserdes_frame[1]),
        .Q3       (iserdes_frame[2]),
        .Q4       (iserdes_frame[3]),
        .Q5       (iserdes_frame[4]),
        .Q6       (iserdes_frame[5]),
        .Q7       (),
        .Q8       (),
        .CLK      (lvds_clk_bufio),
        .CLKB     (~lvds_clk_bufio),
        .CLKDIV   (lvds_clk_bufr),
        .CE1      (1'b1),
        .CE2      (1'b1),
        .RST      (rst),
        .D        (lvds_frame_ibuf),
        .DDLY     (1'b0),
        .BITSLIP  (bitslip_frame),
        .DYNCLKDIVSEL (1'b0),
        .DYNCLKSEL    (1'b0),
        .OFB      (1'b0),
        .OCLK     (1'b0),
        .OCLKB    (1'b0),
        .SHIFTIN1 (1'b0),
        .SHIFTIN2 (1'b0),
        .SHIFTOUT1 (),
        .SHIFTOUT2 (),
        .O        ()
    );

    // Data lane ISERDES (generate for 4 channels)
    generate
        for (g = 0; g < 4; g = g + 1) begin : gen_iserdes_data
            ISERDESE2 #(
                .DATA_RATE       ("DDR"),
                .DATA_WIDTH      (6),
                .INTERFACE_TYPE  ("NETWORKING"),
                .NUM_CE          (1),
                .SERDES_MODE     ("MASTER"),
                .IOBDELAY        ("NONE")
            ) u_iserdes_data (
                .Q1       (iserdes_data[g][0]),
                .Q2       (iserdes_data[g][1]),
                .Q3       (iserdes_data[g][2]),
                .Q4       (iserdes_data[g][3]),
                .Q5       (iserdes_data[g][4]),
                .Q6       (iserdes_data[g][5]),
                .Q7       (),
                .Q8       (),
                .CLK      (lvds_clk_bufio),
                .CLKB     (~lvds_clk_bufio),
                .CLKDIV   (lvds_clk_bufr),
                .CE1      (1'b1),
                .CE2      (1'b1),
                .RST      (rst),
                .D        (lvds_data_ibuf[g]),
                .DDLY     (1'b0),
                .BITSLIP  (bitslip[g]),
                .DYNCLKDIVSEL (1'b0),
                .DYNCLKSEL    (1'b0),
                .OFB      (1'b0),
                .OCLK     (1'b0),
                .OCLKB    (1'b0),
                .SHIFTIN1 (1'b0),
                .SHIFTIN2 (1'b0),
                .SHIFTOUT1 (),
                .SHIFTOUT2 (),
                .O        ()
            );
        end
    endgenerate

    //======================================================================
    // 12-bit Sample Assembly
    // Two consecutive 6-bit ISERDES outputs form one 12-bit ADC sample
    //======================================================================
    reg        phase;                    // 0=first half, 1=second half
    reg [5:0]  data_hi [0:3];           // Upper 6 bits per channel
    reg [11:0] sample [0:3];            // Assembled 12-bit samples
    reg        sample_valid;             // Assembled sample is valid
    reg [5:0]  frame_hi;                // Upper 6 bits of frame pattern
    reg [11:0] frame_pattern;           // Assembled frame pattern

    always @(posedge lvds_clk_bufr) begin
        if (rst) begin
            phase        <= 1'b0;
            sample_valid <= 1'b0;
        end else if (enable) begin
            if (!phase) begin
                // First half: latch upper 6 bits
                data_hi[0] <= iserdes_data[0];
                data_hi[1] <= iserdes_data[1];
                data_hi[2] <= iserdes_data[2];
                data_hi[3] <= iserdes_data[3];
                frame_hi   <= iserdes_frame;
                phase      <= 1'b1;
                sample_valid <= 1'b0;
            end else begin
                // Second half: combine with lower 6 bits
                sample[0]     <= {data_hi[0], iserdes_data[0]};
                sample[1]     <= {data_hi[1], iserdes_data[1]};
                sample[2]     <= {data_hi[2], iserdes_data[2]};
                sample[3]     <= {data_hi[3], iserdes_data[3]};
                frame_pattern <= {frame_hi, iserdes_frame};
                phase         <= 1'b0;
                sample_valid  <= 1'b1;
            end
        end else begin
            phase        <= 1'b0;
            sample_valid <= 1'b0;
        end
    end

    //======================================================================
    // Bitslip Alignment Controller
    // Monitors frame pattern and issues bitslip pulses until alignment
    //======================================================================
    localparam [2:0] AL_IDLE    = 3'd0,
                     AL_CHECK   = 3'd1,
                     AL_SLIP    = 3'd2,
                     AL_WAIT    = 3'd3,
                     AL_ALIGNED = 3'd4;

    reg [2:0]  align_state;
    reg [3:0]  align_count;              // Consecutive good patterns
    reg [3:0]  slip_count;               // Number of bitslips issued
    reg [7:0]  align_wait;               // Wait counter after bitslip

    always @(posedge lvds_clk_bufr) begin
        if (rst) begin
            align_state  <= AL_IDLE;
            align_count  <= 4'd0;
            slip_count   <= 4'd0;
            align_wait   <= 8'd0;
            bitslip      <= 4'b0000;
            bitslip_frame <= 1'b0;
            aligned      <= 1'b0;
        end else begin
            // Default: clear bitslip pulses
            bitslip       <= 4'b0000;
            bitslip_frame <= 1'b0;

            case (align_state)
                AL_IDLE: begin
                    if (enable) begin
                        align_state <= AL_CHECK;
                        align_count <= 4'd0;
                        slip_count  <= 4'd0;
                    end
                end

                AL_CHECK: begin
                    if (sample_valid) begin
                        if (frame_pattern == FRAME_SYNC_PATTERN ||
                            frame_pattern == ~FRAME_SYNC_PATTERN) begin
                            // Good pattern detected
                            align_count <= align_count + 1'b1;
                            if (align_count >= ALIGN_THRESHOLD) begin
                                align_state <= AL_ALIGNED;
                                aligned     <= 1'b1;
                            end
                        end else begin
                            // Bad pattern - need bitslip
                            align_count <= 4'd0;
                            align_state <= AL_SLIP;
                        end
                    end
                end

                AL_SLIP: begin
                    if (slip_count >= 4'd12) begin
                        // Too many slips - cannot align (error condition)
                        align_state <= AL_IDLE;
                        aligned     <= 1'b0;
                    end else begin
                        // Issue bitslip to all lanes + frame
                        bitslip       <= 4'b1111;
                        bitslip_frame <= 1'b1;
                        slip_count    <= slip_count + 1'b1;
                        align_wait    <= 8'd0;
                        align_state   <= AL_WAIT;
                    end
                end

                AL_WAIT: begin
                    // Wait for bitslip to take effect (several parallel clocks)
                    align_wait <= align_wait + 1'b1;
                    if (align_wait >= 8'd20) begin
                        align_state <= AL_CHECK;
                        align_count <= 4'd0;
                    end
                end

                AL_ALIGNED: begin
                    // Monitor for loss of alignment
                    if (!enable) begin
                        align_state <= AL_IDLE;
                        aligned     <= 1'b0;
                    end
                    // TODO: Optionally monitor frame pattern continuously
                    //       and re-align if pattern is lost
                end

                default: align_state <= AL_IDLE;
            endcase
        end
    end

    //======================================================================
    // Chirp/Frame Boundary Detection
    //======================================================================
    reg frame_pattern_prev_valid;
    reg chirp_detect;
    reg frame_detect;

    always @(posedge lvds_clk_bufr) begin
        if (rst) begin
            chirp_detect <= 1'b0;
            frame_detect <= 1'b0;
        end else if (sample_valid && aligned) begin
            // Chirp start: transition from sync pattern to data
            // Frame sync: all-ones pattern after idle period
            // TODO: Implement actual AWR2243 sync pattern detection.
            //       The specific pattern depends on LVDS configuration.
            chirp_detect <= (frame_pattern == FRAME_SYNC_PATTERN);
            frame_detect <= (frame_pattern == FRAME_SYNC_PATTERN) &&
                           !frame_pattern_prev_valid;
            frame_pattern_prev_valid <= (frame_pattern == FRAME_SYNC_PATTERN);
        end else begin
            chirp_detect <= 1'b0;
            frame_detect <= 1'b0;
        end
    end

    //======================================================================
    // Async FIFO - Clock Domain Crossing
    // Transfers data from LVDS clock domain to processing clock domain
    // Uses Xilinx BRAM-based async FIFO (FIFO18E1 primitive)
    //======================================================================
    // FIFO data packing: 4 channels x 12 bits + control = 50 bits
    // We'll use a 72-bit wide FIFO (nearest BRAM width) for simplicity
    wire [71:0] fifo_din;
    wire [71:0] fifo_dout;
    wire        fifo_wr_en;
    wire        fifo_rd_en;
    wire        fifo_full;
    wire        fifo_empty;
    wire        fifo_valid;

    assign fifo_din = {
        4'd0,                            // [71:68] padding
        chirp_detect,                    // [67]    chirp start flag
        frame_detect,                    // [66]    frame sync flag
        2'd0,                            // [65:64] reserved
        sample[3],                       // [63:52] channel 3
        4'd0,                            // [51:48] padding
        sample[2],                       // [47:36] channel 2
        4'd0,                            // [35:32] padding
        sample[1],                       // [31:20] channel 1
        4'd0,                            // [19:16] padding
        sample[0]                        // [15:4]  channel 0 -- shifted for alignment
        // NOTE: Bit positions adjusted for clarity; actual packing may differ
    };

    assign fifo_wr_en = sample_valid & aligned & enable & ~fifo_full;

    // Xilinx FIFO36E1 instantiation for async FIFO
    // In production, use Vivado FIFO Generator IP for easier configuration
    // This is a template showing the concept:
    //
    // fifo_generator_0 u_async_fifo (
    //     .wr_clk   (lvds_clk_bufr),
    //     .rd_clk   (clk_proc),
    //     .rst      (rst),
    //     .din      (fifo_din),
    //     .wr_en    (fifo_wr_en),
    //     .rd_en    (fifo_rd_en),
    //     .dout     (fifo_dout),
    //     .full     (fifo_full),
    //     .empty    (fifo_empty),
    //     .valid    (fifo_valid)
    // );

    // Simplified async FIFO for simulation (replace with IP in synthesis)
    async_fifo #(
        .DATA_WIDTH (72),
        .ADDR_WIDTH (10)                  // 1024 entries deep
    ) u_async_fifo (
        .wr_clk     (lvds_clk_bufr),
        .rd_clk     (clk_proc),
        .rst        (rst),
        .wr_en      (fifo_wr_en),
        .wr_data    (fifo_din),
        .rd_en      (fifo_rd_en),
        .rd_data    (fifo_dout),
        .full       (fifo_full),
        .empty      (fifo_empty)
    );

    assign fifo_rd_en = ~fifo_empty;

    //======================================================================
    // Output Register Stage (clk_proc domain)
    //======================================================================
    reg fifo_rd_valid;

    always @(posedge clk_proc) begin
        if (rst) begin
            data_valid   <= 1'b0;
            ch0_data     <= 12'd0;
            ch1_data     <= 12'd0;
            ch2_data     <= 12'd0;
            ch3_data     <= 12'd0;
            chirp_start  <= 1'b0;
            frame_sync   <= 1'b0;
            fifo_rd_valid <= 1'b0;
        end else begin
            fifo_rd_valid <= fifo_rd_en;
            data_valid    <= fifo_rd_valid;

            if (fifo_rd_valid) begin
                ch0_data    <= fifo_dout[15:4];
                ch1_data    <= fifo_dout[31:20];
                ch2_data    <= fifo_dout[47:36];
                ch3_data    <= fifo_dout[63:52];
                chirp_start <= fifo_dout[67];
                frame_sync  <= fifo_dout[66];
            end else begin
                data_valid  <= 1'b0;
                chirp_start <= 1'b0;
                frame_sync  <= 1'b0;
            end
        end
    end

    //======================================================================
    // Overflow Detection
    //======================================================================
    // Synchronize fifo_full to clk_sys domain for status reporting
    reg [2:0] overflow_sync;
    always @(posedge clk_sys) begin
        if (rst) begin
            overflow_sync <= 3'b000;
            overflow      <= 1'b0;
        end else begin
            overflow_sync <= {overflow_sync[1:0], fifo_full};
            if (overflow_sync[2])
                overflow <= 1'b1;  // Sticky overflow flag
            if (!enable)
                overflow <= 1'b0;  // Clear on disable
        end
    end

endmodule


//==========================================================================
// Simple Async FIFO (Gray-coded pointers)
// For simulation and as a reference design.
// In production, replace with Xilinx FIFO Generator IP.
//==========================================================================
module async_fifo #(
    parameter DATA_WIDTH = 72,
    parameter ADDR_WIDTH = 10
) (
    input  wire                    wr_clk,
    input  wire                    rd_clk,
    input  wire                    rst,
    input  wire                    wr_en,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    input  wire                    rd_en,
    output reg  [DATA_WIDTH-1:0]   rd_data,
    output wire                    full,
    output wire                    empty
);
    localparam DEPTH = 1 << ADDR_WIDTH;

    // Dual-port memory
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Write pointer (wr_clk domain)
    reg [ADDR_WIDTH:0] wr_ptr_bin, wr_ptr_gray;
    // Read pointer (rd_clk domain)
    reg [ADDR_WIDTH:0] rd_ptr_bin, rd_ptr_gray;

    // Synchronized pointers
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;

    // Binary to Gray conversion
    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] bin;
        bin2gray = bin ^ (bin >> 1);
    endfunction

    // Write logic
    always @(posedge wr_clk or posedge rst) begin
        if (rst) begin
            wr_ptr_bin  <= 0;
            wr_ptr_gray <= 0;
        end else if (wr_en && !full) begin
            mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr_bin  <= wr_ptr_bin + 1;
            wr_ptr_gray <= bin2gray(wr_ptr_bin + 1);
        end
    end

    // Read logic
    always @(posedge rd_clk or posedge rst) begin
        if (rst) begin
            rd_ptr_bin  <= 0;
            rd_ptr_gray <= 0;
            rd_data     <= 0;
        end else if (rd_en && !empty) begin
            rd_data     <= mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
            rd_ptr_bin  <= rd_ptr_bin + 1;
            rd_ptr_gray <= bin2gray(rd_ptr_bin + 1);
        end
    end

    // Synchronize write pointer to read clock domain
    always @(posedge rd_clk or posedge rst) begin
        if (rst) begin
            wr_ptr_gray_sync1 <= 0;
            wr_ptr_gray_sync2 <= 0;
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    // Synchronize read pointer to write clock domain
    always @(posedge wr_clk or posedge rst) begin
        if (rst) begin
            rd_ptr_gray_sync1 <= 0;
            rd_ptr_gray_sync2 <= 0;
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    // Full and empty flags
    assign full  = (wr_ptr_gray == {~rd_ptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                     rd_ptr_gray_sync2[ADDR_WIDTH-2:0]});
    assign empty = (rd_ptr_gray == wr_ptr_gray_sync2);

endmodule
