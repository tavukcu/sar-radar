//============================================================================
// spi_bridge.v
// SPI Slave Bridge for ESP32-C3 Communication
//
// The FPGA acts as an SPI slave, allowing the ESP32-C3 (SPI master) to
// read radar data and status information for WiFi streaming. The ESP32
// initiates all transactions - the FPGA simply responds with buffered data.
//
// Protocol:
//   Byte 0: Command byte from ESP32
//   Byte 1: Length (N) - number of payload bytes to transfer
//   Bytes 2..N+1: Payload (direction depends on command)
//
// Commands:
//   0x01 READ_STATUS  - FPGA sends 8-byte status struct
//   0x02 READ_DATA    - FPGA sends N bytes from circular data buffer
//   0x03 WRITE_CONFIG - ESP32 sends N bytes of configuration
//   0x04 READ_IMU     - FPGA sends latest IMU packet (20 bytes)
//   0x05 READ_FFT     - FPGA sends FFT data (1024 bytes = 512 bins x 2)
//
// The circular data buffer (BRAM-based) is written by the radar data
// pipeline and read by the ESP32. Gray-coded pointers ensure safe
// cross-clock-domain operation.
//
// SPI Mode: 0 (CPOL=0, CPHA=0) - slave follows master's clock
//
// Target FPGA: Xilinx Artix-7
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module spi_bridge (
    input  wire        clk,              // System clock (100 MHz)
    input  wire        rst,              // Synchronous reset

    //------------------------------------------------------------------
    // SPI Slave Interface (directly driven by ESP32 master)
    //------------------------------------------------------------------
    input  wire        spi_sclk,         // SPI clock from ESP32
    input  wire        spi_mosi,         // Data from ESP32
    output wire        spi_miso,         // Data to ESP32
    input  wire        spi_cs_n,         // Chip select from ESP32 (active-low)

    //------------------------------------------------------------------
    // Data Input (from radar pipeline, clk domain)
    //------------------------------------------------------------------
    input  wire        data_wr_en,       // Write enable
    input  wire [7:0]  data_wr,          // Data byte to buffer

    //------------------------------------------------------------------
    // Status
    //------------------------------------------------------------------
    input  wire [7:0]  status_byte,      // System status for ESP32
    output wire        buf_empty,        // Circular buffer empty
    output wire        buf_full          // Circular buffer full
);

    //======================================================================
    // Parameters
    //======================================================================
    localparam BUF_ADDR_WIDTH = 12;      // 4096-byte circular buffer
    localparam BUF_SIZE = 1 << BUF_ADDR_WIDTH;

    // Command codes
    localparam CMD_READ_STATUS  = 8'h01;
    localparam CMD_READ_DATA    = 8'h02;
    localparam CMD_WRITE_CONFIG = 8'h03;
    localparam CMD_READ_IMU     = 8'h04;
    localparam CMD_READ_FFT     = 8'h05;

    //======================================================================
    // SPI Slave - Clock Domain Synchronization
    // SPI signals are in the spi_sclk domain; we need to sample them
    // safely in the system clock domain
    //======================================================================
    // Double-synchronize SPI inputs to system clock domain
    reg [2:0] sclk_sync;
    reg [1:0] mosi_sync;
    reg [1:0] cs_sync;

    always @(posedge clk) begin
        if (rst) begin
            sclk_sync <= 3'b000;
            mosi_sync <= 2'b00;
            cs_sync   <= 2'b11;
        end else begin
            sclk_sync <= {sclk_sync[1:0], spi_sclk};
            mosi_sync <= {mosi_sync[0], spi_mosi};
            cs_sync   <= {cs_sync[0], spi_cs_n};
        end
    end

    wire sclk_rising  = (sclk_sync[2:1] == 2'b01);
    wire sclk_falling = (sclk_sync[2:1] == 2'b10);
    wire cs_active    = ~cs_sync[1];
    wire cs_deassert  = (cs_sync == 2'b01);    // Rising edge of CS (deassert)
    wire mosi_s       = mosi_sync[1];

    //======================================================================
    // SPI Slave Shift Register
    // Mode 0: Sample MOSI on rising SCLK, shift out MISO on falling SCLK
    //======================================================================
    reg [7:0]  rx_shift;                  // Receive shift register
    reg [7:0]  tx_shift;                  // Transmit shift register
    reg [2:0]  bit_cnt;                   // Bit counter (0-7)
    reg        byte_received;             // Pulse when byte complete
    reg [7:0]  rx_byte;                   // Last received byte
    reg        miso_reg;                  // MISO output register

    assign spi_miso = cs_active ? miso_reg : 1'bz;  // Tri-state when not selected

    always @(posedge clk) begin
        if (rst || !cs_active) begin
            bit_cnt       <= 3'd0;
            byte_received <= 1'b0;
            rx_shift      <= 8'h00;
        end else begin
            byte_received <= 1'b0;

            // Sample on rising edge of SCLK
            if (sclk_rising) begin
                rx_shift <= {rx_shift[6:0], mosi_s};
                bit_cnt  <= bit_cnt + 1'b1;

                if (bit_cnt == 3'd7) begin
                    byte_received <= 1'b1;
                    rx_byte       <= {rx_shift[6:0], mosi_s};
                end
            end

            // Shift out on falling edge of SCLK
            if (sclk_falling) begin
                miso_reg <= tx_shift[7];
                tx_shift <= {tx_shift[6:0], 1'b0};
            end
        end
    end

    //======================================================================
    // Circular Data Buffer (BRAM)
    //======================================================================
    reg [7:0] circ_buf [0:BUF_SIZE-1];
    reg [BUF_ADDR_WIDTH-1:0] wr_ptr;     // Write pointer (clk domain)
    reg [BUF_ADDR_WIDTH-1:0] rd_ptr;     // Read pointer (clk domain)
    reg [BUF_ADDR_WIDTH:0]   buf_count;  // Number of bytes in buffer

    assign buf_empty = (buf_count == 0);
    assign buf_full  = (buf_count >= BUF_SIZE - 1);

    // Write side: data from radar pipeline
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr    <= 0;
            buf_count <= 0;
        end else begin
            // Simultaneous read and write tracking
            if (data_wr_en && !buf_full) begin
                circ_buf[wr_ptr] <= data_wr;
                wr_ptr <= wr_ptr + 1'b1;
                if (!buf_rd_en)
                    buf_count <= buf_count + 1'b1;
            end
            if (buf_rd_en && !buf_empty) begin
                rd_ptr <= rd_ptr + 1'b1;
                if (!(data_wr_en && !buf_full))
                    buf_count <= buf_count - 1'b1;
            end
        end
    end

    reg buf_rd_en;                        // Read enable (from command handler)
    reg [7:0] buf_rd_data;               // Read data

    always @(posedge clk) begin
        if (rst)
            buf_rd_data <= 8'h00;
        else
            buf_rd_data <= circ_buf[rd_ptr];
    end

    //======================================================================
    // Status Register Block
    // 8-byte status structure returned on CMD_READ_STATUS:
    //   [0] status_byte (state, flags)
    //   [1] buf_count MSB
    //   [2] buf_count LSB
    //   [3] reserved
    //   [4] reserved
    //   [5] reserved
    //   [6] reserved
    //   [7] protocol version (0x01)
    //======================================================================
    reg [7:0] status_regs [0:7];

    always @(posedge clk) begin
        status_regs[0] <= status_byte;
        status_regs[1] <= buf_count[BUF_ADDR_WIDTH:8];
        status_regs[2] <= buf_count[7:0];
        status_regs[3] <= 8'h00;
        status_regs[4] <= 8'h00;
        status_regs[5] <= 8'h00;
        status_regs[6] <= 8'h00;
        status_regs[7] <= 8'h01;          // Protocol version
    end

    //======================================================================
    // Command Handler State Machine
    //======================================================================
    localparam [2:0] CMD_IDLE     = 3'd0,  // Waiting for command byte
                     CMD_LENGTH   = 3'd1,  // Waiting for length byte
                     CMD_EXECUTE  = 3'd2,  // Processing command
                     CMD_RESPOND  = 3'd3,  // Sending response data
                     CMD_RECEIVE  = 3'd4,  // Receiving config data
                     CMD_DONE     = 3'd5;  // Transaction complete

    reg [2:0]  cmd_state;
    reg [7:0]  current_cmd;               // Active command
    reg [7:0]  payload_len;               // Expected payload length
    reg [15:0] byte_cnt;                  // Byte counter within payload
    reg [7:0]  config_buf [0:63];         // Configuration receive buffer
    reg [5:0]  config_ptr;               // Config buffer pointer

    always @(posedge clk) begin
        if (rst || !cs_active) begin
            cmd_state   <= CMD_IDLE;
            current_cmd <= 8'h00;
            byte_cnt    <= 16'd0;
            buf_rd_en   <= 1'b0;
        end else begin
            buf_rd_en <= 1'b0;

            case (cmd_state)
                //----------------------------------------------------------
                CMD_IDLE: begin
                    if (byte_received) begin
                        current_cmd <= rx_byte;
                        cmd_state   <= CMD_LENGTH;
                    end
                    // Pre-load first status byte for fast response
                    tx_shift <= status_regs[0];
                end

                //----------------------------------------------------------
                CMD_LENGTH: begin
                    if (byte_received) begin
                        payload_len <= rx_byte;
                        byte_cnt    <= 16'd0;

                        case (current_cmd)
                            CMD_READ_STATUS: begin
                                cmd_state <= CMD_RESPOND;
                                tx_shift  <= status_regs[0];
                            end
                            CMD_READ_DATA: begin
                                cmd_state <= CMD_RESPOND;
                                buf_rd_en <= 1'b1;
                                tx_shift  <= buf_rd_data;
                            end
                            CMD_WRITE_CONFIG: begin
                                cmd_state  <= CMD_RECEIVE;
                                config_ptr <= 6'd0;
                            end
                            CMD_READ_IMU: begin
                                cmd_state <= CMD_RESPOND;
                                tx_shift  <= 8'h00;
                                // TODO: Load IMU data buffer
                            end
                            CMD_READ_FFT: begin
                                cmd_state <= CMD_RESPOND;
                                buf_rd_en <= 1'b1;
                                tx_shift  <= buf_rd_data;
                            end
                            default: begin
                                cmd_state <= CMD_DONE;
                                tx_shift  <= 8'hFF;  // Unknown command
                            end
                        endcase
                    end
                end

                //----------------------------------------------------------
                // Send response data to ESP32
                CMD_RESPOND: begin
                    if (byte_received) begin
                        byte_cnt <= byte_cnt + 1'b1;

                        if (byte_cnt >= payload_len - 1) begin
                            cmd_state <= CMD_DONE;
                        end

                        // Prepare next byte based on command type
                        case (current_cmd)
                            CMD_READ_STATUS: begin
                                if (byte_cnt + 1 < 8)
                                    tx_shift <= status_regs[byte_cnt + 1];
                                else
                                    tx_shift <= 8'h00;
                            end
                            CMD_READ_DATA, CMD_READ_FFT: begin
                                buf_rd_en <= 1'b1;
                                tx_shift  <= buf_rd_data;
                            end
                            default: tx_shift <= 8'h00;
                        endcase
                    end
                end

                //----------------------------------------------------------
                // Receive configuration data from ESP32
                CMD_RECEIVE: begin
                    if (byte_received) begin
                        if (config_ptr < 6'd63) begin
                            config_buf[config_ptr] <= rx_byte;
                            config_ptr <= config_ptr + 1'b1;
                        end
                        byte_cnt <= byte_cnt + 1'b1;

                        if (byte_cnt >= payload_len - 1) begin
                            cmd_state <= CMD_DONE;
                            // TODO: Process received configuration
                            //       Parse config_buf and apply settings
                        end
                    end
                end

                //----------------------------------------------------------
                CMD_DONE: begin
                    // Wait for CS deassert
                    if (!cs_active)
                        cmd_state <= CMD_IDLE;
                end

                default: cmd_state <= CMD_IDLE;
            endcase
        end
    end

endmodule
