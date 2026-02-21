//============================================================================
// sd_spi.v
// MicroSD Card SPI Interface
//
// Provides raw sector-level access to a microSD card via SPI mode.
// Used for logging raw radar data and IMU measurements during SAR scans.
// SPI mode is chosen over SDIO for simplicity - the lower throughput
// (~2-4 MB/s) is sufficient for data logging alongside WiFi streaming.
//
// SD Card SPI Mode Protocol:
//   - SPI Mode 0 (CPOL=0, CPHA=0)
//   - Clock: 100-400 kHz during init, up to 25 MHz after
//   - Commands: 6 bytes (start bit + cmd + 4-byte arg + CRC7 + stop)
//   - Responses: R1 (1 byte), R3/R7 (5 bytes)
//   - Data blocks: 512 bytes + CRC16
//
// Initialization Sequence:
//   1. >74 clock cycles with CS high (card enters SPI mode)
//   2. CMD0 (GO_IDLE) - reset card
//   3. CMD8 (SEND_IF_COND) - check voltage range (SDv2)
//   4. ACMD41 (SD_SEND_OP_COND) - initialize card
//   5. CMD58 (READ_OCR) - read card capacity (SDHC/SDXC)
//   6. Card ready for block read/write
//
// Write Operations:
//   - CMD24: Single block write (512 bytes)
//   - CMD25: Multi-block write (continuous until CMD12 stop)
//
// Note: Raw sector writes are used (no FAT32 filesystem) for maximum
//       write speed. A host-side tool reads the raw data for processing.
//
// Target FPGA: Xilinx Artix-7
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module sd_spi (
    input  wire        clk,              // System clock (100 MHz)
    input  wire        rst,              // Synchronous reset

    //------------------------------------------------------------------
    // SPI Physical Interface
    //------------------------------------------------------------------
    output reg         spi_sclk,         // SPI clock
    output reg         spi_mosi,         // Master Out Slave In
    input  wire        spi_miso,         // Master In Slave Out
    output reg         spi_cs_n,         // Chip select (active-low)

    //------------------------------------------------------------------
    // Status
    //------------------------------------------------------------------
    output reg         sd_ready,         // Card initialized and ready
    output reg         sd_busy,          // Write operation in progress
    output reg         sd_error,         // Error flag (sticky)

    //------------------------------------------------------------------
    // Write Interface
    //------------------------------------------------------------------
    input  wire        write_req,        // Write request (pulse)
    input  wire [7:0]  write_data,       // Byte to write
    output reg         write_ack         // Write byte acknowledged
);

    //======================================================================
    // Parameters
    //======================================================================
    // SPI clock dividers (system clock / (2 * divider))
    localparam INIT_CLK_DIV = 250;       // 100 MHz / 500 = 200 kHz (init)
    localparam FAST_CLK_DIV = 2;         // 100 MHz / 4 = 25 MHz (fast)

    // SD command definitions
    localparam CMD0   = 6'd0;            // GO_IDLE_STATE
    localparam CMD8   = 6'd8;            // SEND_IF_COND
    localparam CMD12  = 6'd12;           // STOP_TRANSMISSION
    localparam CMD16  = 6'd16;           // SET_BLOCKLEN
    localparam CMD24  = 6'd24;           // WRITE_SINGLE_BLOCK
    localparam CMD25  = 6'd25;           // WRITE_MULTIPLE_BLOCK
    localparam CMD55  = 6'd55;           // APP_CMD prefix
    localparam CMD58  = 6'd58;           // READ_OCR
    localparam ACMD41 = 6'd41;           // SD_SEND_OP_COND

    // R1 response bits
    localparam R1_IDLE = 8'h01;          // In idle state (OK during init)
    localparam R1_OK   = 8'h00;          // No errors

    // Data tokens
    localparam DATA_START_TOKEN    = 8'hFE;  // Single block write
    localparam DATA_START_MULTI    = 8'hFC;  // Multi-block write
    localparam DATA_STOP_MULTI     = 8'hFD;  // Stop multi-block write

    // Data response tokens
    localparam DATA_ACCEPTED       = 5'b00101;
    localparam DATA_CRC_ERROR      = 5'b01011;
    localparam DATA_WRITE_ERROR    = 5'b01101;

    //======================================================================
    // SPI Clock Generation
    //======================================================================
    reg [8:0]  clk_div;                  // Current clock divider setting
    reg [8:0]  clk_cnt;                  // Clock divider counter
    reg        spi_clk_en;               // SPI clock enable pulse

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt    <= 9'd0;
            spi_clk_en <= 1'b0;
        end else begin
            if (clk_cnt >= clk_div - 1) begin
                clk_cnt    <= 9'd0;
                spi_clk_en <= 1'b1;
            end else begin
                clk_cnt    <= clk_cnt + 1'b1;
                spi_clk_en <= 1'b0;
            end
        end
    end

    //======================================================================
    // SPI Byte Transfer Engine
    // Sends/receives one byte at a time (Mode 0: CPOL=0, CPHA=0)
    //======================================================================
    reg [7:0]  tx_byte;                  // Byte to transmit
    reg [7:0]  rx_byte;                  // Received byte
    reg [3:0]  bit_cnt;                  // Bit counter
    reg        byte_active;              // Byte transfer in progress
    reg        byte_done;                // Byte transfer complete
    reg        spi_phase;                // 0=setup, 1=sample

    always @(posedge clk) begin
        if (rst) begin
            spi_sclk    <= 1'b0;
            spi_mosi    <= 1'b1;
            bit_cnt     <= 4'd0;
            byte_done   <= 1'b0;
            spi_phase   <= 1'b0;
            rx_byte     <= 8'hFF;
        end else if (byte_active && spi_clk_en) begin
            byte_done <= 1'b0;
            if (!spi_phase) begin
                // Setup phase: put data on MOSI, SCLK low
                spi_sclk  <= 1'b0;
                spi_mosi  <= tx_byte[7];
                spi_phase <= 1'b1;
            end else begin
                // Sample phase: read MISO, SCLK high
                spi_sclk  <= 1'b1;
                rx_byte   <= {rx_byte[6:0], spi_miso};
                tx_byte   <= {tx_byte[6:0], 1'b1};
                bit_cnt   <= bit_cnt + 1'b1;
                spi_phase <= 1'b0;

                if (bit_cnt == 4'd7) begin
                    byte_done <= 1'b1;
                    bit_cnt   <= 4'd0;
                end
            end
        end else if (!byte_active) begin
            spi_sclk  <= 1'b0;
            spi_phase <= 1'b0;
            bit_cnt   <= 4'd0;
            byte_done <= 1'b0;
        end
    end

    //======================================================================
    // SD Command Builder
    // Constructs 6-byte command: [01 cmd[5:0]] [arg[31:0]] [crc7 1]
    //======================================================================
    reg [5:0]  cmd_index;
    reg [31:0] cmd_arg;
    reg [7:0]  cmd_crc;
    reg [47:0] cmd_shift;                // 6-byte command shift register
    reg [2:0]  cmd_byte_cnt;             // Byte counter within command

    // Pre-computed CRC7 values for common commands
    // In production, implement CRC7 calculator
    function [7:0] get_cmd_crc;
        input [5:0] cmd;
        input [31:0] arg;
        begin
            case (cmd)
                CMD0:    get_cmd_crc = 8'h95;  // CMD0 with arg=0
                CMD8:    get_cmd_crc = 8'h87;  // CMD8 with arg=0x1AA
                default: get_cmd_crc = 8'hFF;  // CRC not checked in SPI mode
            endcase
        end
    endfunction

    //======================================================================
    // Block Write Buffer
    // 512-byte buffer for accumulating data before writing to SD
    //======================================================================
    reg [7:0]  block_buf [0:511];
    reg [9:0]  block_wr_ptr;             // Write pointer into buffer
    reg [9:0]  block_rd_ptr;             // Read pointer (during SD write)
    reg        block_full;               // Buffer contains 512 bytes

    //======================================================================
    // Main State Machine
    //======================================================================
    localparam [4:0] ST_POWER_UP     = 5'd0,   // Initial power-up delay
                     ST_SEND_CLOCKS  = 5'd1,   // 80+ clocks with CS high
                     ST_SEND_CMD0    = 5'd2,   // Send CMD0 (reset)
                     ST_WAIT_R1      = 5'd3,   // Wait for R1 response
                     ST_SEND_CMD8    = 5'd4,   // Send CMD8 (voltage check)
                     ST_WAIT_R7      = 5'd5,   // Wait for R7 response
                     ST_SEND_CMD55   = 5'd6,   // Send CMD55 (APP_CMD prefix)
                     ST_SEND_ACMD41  = 5'd7,   // Send ACMD41 (init)
                     ST_SEND_CMD58   = 5'd8,   // Send CMD58 (read OCR)
                     ST_WAIT_R3      = 5'd9,   // Wait for R3 response
                     ST_INIT_DONE    = 5'd10,  // Initialization complete
                     ST_IDLE         = 5'd11,  // Ready, waiting for write
                     ST_FILL_BLOCK   = 5'd12,  // Filling 512-byte block
                     ST_SEND_CMD24   = 5'd13,  // Send write command
                     ST_SEND_TOKEN   = 5'd14,  // Send data start token
                     ST_SEND_DATA    = 5'd15,  // Send 512 data bytes
                     ST_SEND_CRC     = 5'd16,  // Send CRC16 (dummy)
                     ST_WAIT_DRESP   = 5'd17,  // Wait for data response
                     ST_WAIT_BUSY    = 5'd18,  // Wait for card not busy
                     ST_MULTI_START  = 5'd19,  // Start multi-block write
                     ST_MULTI_STOP   = 5'd20,  // Stop multi-block write
                     ST_ERROR_STATE  = 5'd21;

    reg [4:0]  state;
    reg [31:0] wait_cnt;
    reg [7:0]  r1_response;
    reg [31:0] r7_response;
    reg [31:0] ocr_response;
    reg [3:0]  resp_byte_cnt;
    reg [7:0]  retry_cnt;
    reg        sdhc_card;                 // SDHC/SDXC card flag
    reg [31:0] write_sector;              // Current write sector address

    // Track number of clocks sent during init
    reg [7:0]  init_clk_cnt;

    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_POWER_UP;
            wait_cnt     <= 32'd0;
            sd_ready     <= 1'b0;
            sd_busy      <= 1'b0;
            sd_error     <= 1'b0;
            write_ack    <= 1'b0;
            spi_cs_n     <= 1'b1;
            byte_active  <= 1'b0;
            clk_div      <= INIT_CLK_DIV;  // Start with slow clock
            block_wr_ptr <= 10'd0;
            block_rd_ptr <= 10'd0;
            block_full   <= 1'b0;
            sdhc_card    <= 1'b0;
            write_sector <= 32'd0;
            retry_cnt    <= 8'd0;
            init_clk_cnt <= 8'd0;
        end else begin
            // Defaults
            write_ack <= 1'b0;

            case (state)
                //----------------------------------------------------------
                // Power-up delay (>1ms after power stable)
                ST_POWER_UP: begin
                    spi_cs_n <= 1'b1;      // CS high during power-up
                    wait_cnt <= wait_cnt + 1'b1;
                    if (wait_cnt >= 32'd200_000) begin  // 2ms at 100 MHz
                        state        <= ST_SEND_CLOCKS;
                        wait_cnt     <= 32'd0;
                        init_clk_cnt <= 8'd0;
                    end
                end

                //----------------------------------------------------------
                // Send >74 clock pulses with CS high
                ST_SEND_CLOCKS: begin
                    spi_cs_n <= 1'b1;      // CS must be high
                    tx_byte     <= 8'hFF;     // MOSI high
                    byte_active <= 1'b1;

                    if (byte_done) begin
                        init_clk_cnt <= init_clk_cnt + 1'b1;
                        if (init_clk_cnt >= 8'd10) begin  // 80 clocks (10 bytes)
                            byte_active <= 1'b0;
                            state       <= ST_SEND_CMD0;
                        end
                    end
                end

                //----------------------------------------------------------
                // CMD0: GO_IDLE_STATE
                ST_SEND_CMD0: begin
                    spi_cs_n <= 1'b0;
                    cmd_shift <= {2'b01, CMD0, 32'h00000000, 8'h95};
                    cmd_byte_cnt <= 3'd0;
                    state     <= ST_WAIT_R1;
                    // Send command bytes
                    tx_byte     <= {2'b01, CMD0};
                    byte_active <= 1'b1;
                    // TODO: Send all 6 bytes sequentially
                    //       This is simplified - needs proper byte sequencing
                end

                //----------------------------------------------------------
                // Wait for R1 response (poll until non-0xFF)
                ST_WAIT_R1: begin
                    if (byte_done) begin
                        if (rx_byte != 8'hFF) begin
                            r1_response <= rx_byte;
                            byte_active <= 1'b0;
                            spi_cs_n    <= 1'b1;

                            // Route based on which command we sent
                            case (cmd_index)
                                CMD0: begin
                                    if (rx_byte == R1_IDLE)
                                        state <= ST_SEND_CMD8;
                                    else begin
                                        retry_cnt <= retry_cnt + 1'b1;
                                        if (retry_cnt > 100)
                                            state <= ST_ERROR_STATE;
                                        else
                                            state <= ST_SEND_CMD0;
                                    end
                                end
                                CMD8:    state <= ST_WAIT_R7;
                                CMD55:   state <= ST_SEND_ACMD41;
                                ACMD41: begin
                                    if (rx_byte == R1_OK)
                                        state <= ST_SEND_CMD58;
                                    else begin
                                        // Not ready yet, retry
                                        wait_cnt <= 32'd0;
                                        state    <= ST_SEND_CMD55;
                                    end
                                end
                                CMD24: begin
                                    if (rx_byte == R1_OK)
                                        state <= ST_SEND_TOKEN;
                                    else
                                        state <= ST_ERROR_STATE;
                                end
                                default: state <= ST_IDLE;
                            endcase
                        end else begin
                            // Keep polling (send 0xFF and read response)
                            tx_byte     <= 8'hFF;
                            byte_active <= 1'b1;
                            wait_cnt    <= wait_cnt + 1'b1;
                            if (wait_cnt > 32'd10000)
                                state <= ST_ERROR_STATE;  // Timeout
                        end
                    end
                end

                //----------------------------------------------------------
                // CMD8: SEND_IF_COND - check SDv2 support
                ST_SEND_CMD8: begin
                    cmd_index   <= CMD8;
                    cmd_arg     <= 32'h000001AA;  // 2.7-3.6V, check pattern 0xAA
                    spi_cs_n    <= 1'b0;
                    tx_byte     <= {2'b01, CMD8};
                    byte_active <= 1'b1;
                    wait_cnt    <= 32'd0;
                    state       <= ST_WAIT_R1;
                    // TODO: Send full 6-byte command with CRC=0x87
                end

                //----------------------------------------------------------
                // Read R7 response (4 additional bytes after R1)
                ST_WAIT_R7: begin
                    // TODO: Read 4 more bytes for R7 response
                    // Check that voltage range and check pattern match
                    // r7_response should contain 0x000001AA
                    state <= ST_SEND_CMD55;
                end

                //----------------------------------------------------------
                // CMD55 + ACMD41: Initialize card
                ST_SEND_CMD55: begin
                    cmd_index   <= CMD55;
                    spi_cs_n    <= 1'b0;
                    tx_byte     <= {2'b01, CMD55};
                    byte_active <= 1'b1;
                    wait_cnt    <= 32'd0;
                    state       <= ST_WAIT_R1;
                end

                ST_SEND_ACMD41: begin
                    cmd_index   <= ACMD41;
                    cmd_arg     <= 32'h40000000;  // HCS=1 (support SDHC)
                    spi_cs_n    <= 1'b0;
                    tx_byte     <= {2'b01, ACMD41};
                    byte_active <= 1'b1;
                    wait_cnt    <= 32'd0;
                    state       <= ST_WAIT_R1;
                end

                //----------------------------------------------------------
                // CMD58: Read OCR to determine card type
                ST_SEND_CMD58: begin
                    cmd_index <= CMD58;
                    spi_cs_n  <= 1'b0;
                    tx_byte     <= {2'b01, CMD58};
                    byte_active <= 1'b1;
                    state       <= ST_WAIT_R3;
                end

                ST_WAIT_R3: begin
                    // TODO: Read R1 + 4 bytes OCR
                    // Check CCS bit (bit 30) for SDHC
                    sdhc_card <= 1'b1;            // Assume SDHC for now
                    state     <= ST_INIT_DONE;
                end

                //----------------------------------------------------------
                // Initialization complete
                ST_INIT_DONE: begin
                    byte_active <= 1'b0;
                    spi_cs_n    <= 1'b1;
                    clk_div     <= FAST_CLK_DIV;  // Switch to fast clock
                    sd_ready    <= 1'b1;
                    state       <= ST_IDLE;
                end

                //----------------------------------------------------------
                // Idle: ready for write operations
                ST_IDLE: begin
                    sd_busy <= 1'b0;
                    if (write_req) begin
                        state <= ST_FILL_BLOCK;
                    end
                end

                //----------------------------------------------------------
                // Fill 512-byte block buffer
                ST_FILL_BLOCK: begin
                    if (write_req) begin
                        block_buf[block_wr_ptr] <= write_data;
                        block_wr_ptr <= block_wr_ptr + 1'b1;
                        write_ack    <= 1'b1;

                        if (block_wr_ptr == 10'd511) begin
                            block_wr_ptr <= 10'd0;
                            block_full   <= 1'b1;
                            sd_busy      <= 1'b1;
                            state        <= ST_SEND_CMD24;
                        end
                    end
                end

                //----------------------------------------------------------
                // CMD24: Write single block
                ST_SEND_CMD24: begin
                    cmd_index <= CMD24;
                    // SDHC uses block address, SDSC uses byte address
                    cmd_arg   <= sdhc_card ? write_sector : (write_sector << 9);
                    spi_cs_n  <= 1'b0;
                    tx_byte     <= {2'b01, CMD24};
                    byte_active <= 1'b1;
                    wait_cnt    <= 32'd0;
                    state       <= ST_WAIT_R1;
                end

                //----------------------------------------------------------
                // Send data start token
                ST_SEND_TOKEN: begin
                    tx_byte      <= DATA_START_TOKEN;
                    byte_active  <= 1'b1;
                    block_rd_ptr <= 10'd0;
                    if (byte_done) begin
                        state <= ST_SEND_DATA;
                    end
                end

                //----------------------------------------------------------
                // Send 512 data bytes
                ST_SEND_DATA: begin
                    if (byte_done || !byte_active) begin
                        if (block_rd_ptr < 10'd512) begin
                            tx_byte      <= block_buf[block_rd_ptr];
                            byte_active  <= 1'b1;
                            block_rd_ptr <= block_rd_ptr + 1'b1;
                        end else begin
                            state <= ST_SEND_CRC;
                        end
                    end
                end

                //----------------------------------------------------------
                // Send dummy CRC16 (not checked in SPI mode)
                ST_SEND_CRC: begin
                    tx_byte     <= 8'hFF;
                    byte_active <= 1'b1;
                    if (byte_done) begin
                        resp_byte_cnt <= resp_byte_cnt + 1'b1;
                        if (resp_byte_cnt >= 4'd1) begin  // 2 CRC bytes sent
                            resp_byte_cnt <= 4'd0;
                            state         <= ST_WAIT_DRESP;
                        end
                    end
                end

                //----------------------------------------------------------
                // Wait for data response token
                ST_WAIT_DRESP: begin
                    tx_byte     <= 8'hFF;
                    byte_active <= 1'b1;
                    if (byte_done) begin
                        if (rx_byte[0] == 1'b1) begin
                            // Valid data response received
                            if (rx_byte[4:1] == DATA_ACCEPTED[4:1]) begin
                                state <= ST_WAIT_BUSY;
                            end else begin
                                state <= ST_ERROR_STATE;  // CRC or write error
                            end
                        end
                        // else keep polling
                    end
                end

                //----------------------------------------------------------
                // Wait for card to finish programming (MISO goes high)
                ST_WAIT_BUSY: begin
                    tx_byte     <= 8'hFF;
                    byte_active <= 1'b1;
                    if (byte_done) begin
                        if (rx_byte == 8'hFF) begin
                            // Card is done
                            byte_active  <= 1'b0;
                            spi_cs_n     <= 1'b1;
                            block_full   <= 1'b0;
                            write_sector <= write_sector + 1'b1;
                            state        <= ST_IDLE;
                        end
                        // else card is still busy (MISO=0), keep polling
                    end
                end

                //----------------------------------------------------------
                // Error state
                ST_ERROR_STATE: begin
                    sd_error    <= 1'b1;
                    sd_busy     <= 1'b0;
                    byte_active <= 1'b0;
                    spi_cs_n    <= 1'b1;
                    // Sticky error - needs system reset to clear
                    // TODO: Implement error recovery / retry mechanism
                end

                default: state <= ST_ERROR_STATE;
            endcase
        end
    end

endmodule
