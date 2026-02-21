//============================================================================
// display_spi.v
// ST7789 TFT Display SPI Driver
//
// Drives a small ST7789V-based TFT display (typically 240x135 or 240x240)
// via SPI to show SAR radar system status information. This is a low-priority
// peripheral that updates at ~10 Hz with status text and simple graphics.
//
// ST7789V SPI Interface:
//   - SPI Mode 0 (CPOL=0, CPHA=0)
//   - Max SPI clock: 60 MHz (write-only, no MISO needed)
//   - D/C pin: LOW = command byte, HIGH = data bytes
//   - CS pin: active-low chip select
//   - RST pin: active-low hardware reset
//
// Display Content:
//   - Line 1: "SAR RADAR" + mode (SAR/Range/Raw)
//   - Line 2: State (IDLE/CONFIG/SCAN/SAVE/ERR)
//   - Line 3: Frame count
//   - Line 4: WiFi + SD status indicators
//
// Implementation:
//   - Simple 8x8 pixel monospace font stored in ROM (128 characters)
//   - Text-only rendering for minimal BRAM usage
//   - Character buffer (20 chars x 4 lines = 80 bytes)
//   - Background color-coded by system state
//
// Target FPGA: Xilinx Artix-7
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module display_spi (
    input  wire        clk,              // System clock (100 MHz)
    input  wire        rst,              // Synchronous reset

    //------------------------------------------------------------------
    // SPI Physical Interface (write-only to ST7789)
    //------------------------------------------------------------------
    output reg         spi_sclk,         // SPI clock
    output reg         spi_mosi,         // SPI data (MOSI only)
    output reg         spi_cs_n,         // Chip select (active-low)
    output reg         dc,               // Data/Command select
    output reg         disp_rst_n,       // Display reset (active-low)
    output reg         backlight,        // Backlight enable

    //------------------------------------------------------------------
    // Status Inputs
    //------------------------------------------------------------------
    input  wire        scan_active,      // Scanning in progress
    input  wire [31:0] frame_count,      // Total frames captured
    input  wire        wifi_connected,   // WiFi connection status
    input  wire [2:0]  sd_status,        // {ready, busy, error}
    input  wire [2:0]  system_state,     // Current system state

    //------------------------------------------------------------------
    // Status Outputs
    //------------------------------------------------------------------
    output reg         disp_ready,       // Display initialized
    output reg         disp_busy         // Display update in progress
);

    //======================================================================
    // Parameters
    //======================================================================
    // SPI clock divider: 100 MHz / (2*2) = 25 MHz SPI clock
    localparam SPI_CLK_DIV = 2;

    // Display dimensions (using a small area for text status)
    localparam DISP_WIDTH  = 240;
    localparam DISP_HEIGHT = 135;
    localparam CHAR_WIDTH  = 8;
    localparam CHAR_HEIGHT = 8;
    localparam TEXT_COLS   = 20;          // Characters per line
    localparam TEXT_ROWS   = 4;           // Number of text lines

    // ST7789 Commands
    localparam CMD_SWRESET  = 8'h01;     // Software reset
    localparam CMD_SLPOUT   = 8'h11;     // Sleep out
    localparam CMD_COLMOD   = 8'h3A;     // Color mode
    localparam CMD_MADCTL   = 8'h36;     // Memory access control
    localparam CMD_CASET    = 8'h2A;     // Column address set
    localparam CMD_RASET    = 8'h2B;     // Row address set
    localparam CMD_RAMWR    = 8'h2C;     // Memory write
    localparam CMD_INVON    = 8'h21;     // Inversion on
    localparam CMD_NORON    = 8'h13;     // Normal display on
    localparam CMD_DISPON   = 8'h29;     // Display on
    localparam CMD_GAMSET   = 8'h26;     // Gamma set
    localparam CMD_PVGAMCTRL = 8'hE0;   // Positive voltage gamma
    localparam CMD_NVGAMCTRL = 8'hE1;   // Negative voltage gamma

    // Color definitions (RGB565)
    localparam COLOR_BLACK  = 16'h0000;
    localparam COLOR_WHITE  = 16'hFFFF;
    localparam COLOR_RED    = 16'hF800;
    localparam COLOR_GREEN  = 16'h07E0;
    localparam COLOR_BLUE   = 16'h001F;
    localparam COLOR_YELLOW = 16'hFFE0;
    localparam COLOR_CYAN   = 16'h07FF;

    //======================================================================
    // 8x8 Font ROM
    // Stores bitmap data for printable ASCII characters (32-127)
    // Each character is 8 bytes (8 rows of 8 pixels)
    //======================================================================
    reg [7:0] font_rom [0:767];  // 96 characters x 8 bytes each

    initial begin
        // Initialize with basic font glyphs
        // Only essential characters shown; full font loaded from hex file
        // Format: each byte is one row, MSB = leftmost pixel

        // Space (ASCII 32) - offset 0
        font_rom[0]  = 8'h00; font_rom[1]  = 8'h00; font_rom[2]  = 8'h00;
        font_rom[3]  = 8'h00; font_rom[4]  = 8'h00; font_rom[5]  = 8'h00;
        font_rom[6]  = 8'h00; font_rom[7]  = 8'h00;

        // '0' (ASCII 48) - offset 128
        font_rom[128] = 8'h3C; font_rom[129] = 8'h66; font_rom[130] = 8'h6E;
        font_rom[131] = 8'h76; font_rom[132] = 8'h66; font_rom[133] = 8'h66;
        font_rom[134] = 8'h3C; font_rom[135] = 8'h00;

        // 'A' (ASCII 65) - offset 264
        font_rom[264] = 8'h18; font_rom[265] = 8'h3C; font_rom[266] = 8'h66;
        font_rom[267] = 8'h7E; font_rom[268] = 8'h66; font_rom[269] = 8'h66;
        font_rom[270] = 8'h66; font_rom[271] = 8'h00;

        // 'S' (ASCII 83) - offset 408
        font_rom[408] = 8'h3C; font_rom[409] = 8'h66; font_rom[410] = 8'h30;
        font_rom[411] = 8'h18; font_rom[412] = 8'h0C; font_rom[413] = 8'h66;
        font_rom[414] = 8'h3C; font_rom[415] = 8'h00;

        // 'R' (ASCII 82) - offset 400
        font_rom[400] = 8'h7C; font_rom[401] = 8'h66; font_rom[402] = 8'h66;
        font_rom[403] = 8'h7C; font_rom[404] = 8'h6C; font_rom[405] = 8'h66;
        font_rom[406] = 8'h66; font_rom[407] = 8'h00;

        // TODO: Load complete 8x8 font from hex file:
        // $readmemh("font_8x8.hex", font_rom);
    end

    //======================================================================
    // Character Buffer (text to display)
    //======================================================================
    reg [7:0] char_buf [0:TEXT_COLS*TEXT_ROWS-1]; // 80 characters
    reg       char_buf_dirty;             // Needs refresh

    // Update character buffer from status inputs
    // This converts numeric values to ASCII text
    integer ci;
    reg [31:0] frame_temp;
    reg [3:0]  digit;

    always @(posedge clk) begin
        if (rst) begin
            // Clear buffer with spaces
            for (ci = 0; ci < TEXT_COLS * TEXT_ROWS; ci = ci + 1)
                char_buf[ci] <= 8'h20;  // Space
            char_buf_dirty <= 1'b1;
        end else begin
            // Line 0: "SAR RADAR  [MODE]"
            char_buf[0]  <= "S"; char_buf[1]  <= "A"; char_buf[2]  <= "R";
            char_buf[3]  <= " "; char_buf[4]  <= "R"; char_buf[5]  <= "A";
            char_buf[6]  <= "D"; char_buf[7]  <= "A"; char_buf[8]  <= "R";
            char_buf[9]  <= " "; char_buf[10] <= " ";
            char_buf[11] <= "7"; char_buf[12] <= "7";
            char_buf[13] <= "G"; char_buf[14] <= "H"; char_buf[15] <= "z";
            char_buf[16] <= " "; char_buf[17] <= " "; char_buf[18] <= " ";
            char_buf[19] <= " ";

            // Line 1: System state
            char_buf[20] <= "S"; char_buf[21] <= "T"; char_buf[22] <= ":";
            char_buf[23] <= " ";
            case (system_state)
                3'd0: begin  // IDLE
                    char_buf[24] <= "I"; char_buf[25] <= "D";
                    char_buf[26] <= "L"; char_buf[27] <= "E";
                    char_buf[28] <= " "; char_buf[29] <= " ";
                end
                3'd1: begin  // CONFIG
                    char_buf[24] <= "C"; char_buf[25] <= "O";
                    char_buf[26] <= "N"; char_buf[27] <= "F";
                    char_buf[28] <= "I"; char_buf[29] <= "G";
                end
                3'd2: begin  // SCANNING
                    char_buf[24] <= "S"; char_buf[25] <= "C";
                    char_buf[26] <= "A"; char_buf[27] <= "N";
                    char_buf[28] <= " "; char_buf[29] <= " ";
                end
                3'd3: begin  // SAVING
                    char_buf[24] <= "S"; char_buf[25] <= "A";
                    char_buf[26] <= "V"; char_buf[27] <= "E";
                    char_buf[28] <= " "; char_buf[29] <= " ";
                end
                3'd4: begin  // ERROR
                    char_buf[24] <= "E"; char_buf[25] <= "R";
                    char_buf[26] <= "R"; char_buf[27] <= "O";
                    char_buf[28] <= "R"; char_buf[29] <= " ";
                end
                default: begin
                    char_buf[24] <= "?"; char_buf[25] <= "?";
                    char_buf[26] <= "?"; char_buf[27] <= " ";
                    char_buf[28] <= " "; char_buf[29] <= " ";
                end
            endcase
            char_buf[30] <= " "; char_buf[31] <= " ";
            char_buf[32] <= " "; char_buf[33] <= " ";
            char_buf[34] <= " "; char_buf[35] <= " ";
            char_buf[36] <= " "; char_buf[37] <= " ";
            char_buf[38] <= " "; char_buf[39] <= " ";

            // Line 2: Frame count "FR: XXXXXXXX"
            char_buf[40] <= "F"; char_buf[41] <= "R"; char_buf[42] <= ":";
            char_buf[43] <= " ";
            // Convert frame_count to decimal digits (simplified - hex display)
            char_buf[44] <= hex_to_ascii(frame_count[31:28]);
            char_buf[45] <= hex_to_ascii(frame_count[27:24]);
            char_buf[46] <= hex_to_ascii(frame_count[23:20]);
            char_buf[47] <= hex_to_ascii(frame_count[19:16]);
            char_buf[48] <= hex_to_ascii(frame_count[15:12]);
            char_buf[49] <= hex_to_ascii(frame_count[11:8]);
            char_buf[50] <= hex_to_ascii(frame_count[7:4]);
            char_buf[51] <= hex_to_ascii(frame_count[3:0]);
            char_buf[52] <= " "; char_buf[53] <= " ";
            char_buf[54] <= " "; char_buf[55] <= " ";
            char_buf[56] <= " "; char_buf[57] <= " ";
            char_buf[58] <= " "; char_buf[59] <= " ";

            // Line 3: WiFi + SD status
            char_buf[60] <= "W"; char_buf[61] <= "F"; char_buf[62] <= ":";
            char_buf[63] <= wifi_connected ? "Y" : "N";
            char_buf[64] <= " ";
            char_buf[65] <= "S"; char_buf[66] <= "D"; char_buf[67] <= ":";
            char_buf[68] <= sd_status[2] ? "R" : "-";  // Ready
            char_buf[69] <= sd_status[1] ? "B" : "-";  // Busy
            char_buf[70] <= sd_status[0] ? "E" : "-";  // Error
            char_buf[71] <= " "; char_buf[72] <= " ";
            char_buf[73] <= " "; char_buf[74] <= " ";
            char_buf[75] <= " "; char_buf[76] <= " ";
            char_buf[77] <= " "; char_buf[78] <= " ";
            char_buf[79] <= " ";

            char_buf_dirty <= 1'b1;
        end
    end

    // Hex nibble to ASCII conversion
    function [7:0] hex_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                hex_to_ascii = 8'h30 + nibble;  // '0'-'9'
            else
                hex_to_ascii = 8'h41 + nibble - 4'd10;  // 'A'-'F'
        end
    endfunction

    //======================================================================
    // SPI Byte Transfer Engine
    //======================================================================
    reg [7:0]  tx_byte;
    reg [3:0]  spi_bit_cnt;
    reg        spi_active;
    reg        spi_done;
    reg [7:0]  spi_clk_cnt;
    reg        spi_clk_en;
    reg        spi_phase;

    always @(posedge clk) begin
        if (rst || !spi_active) begin
            spi_clk_cnt <= 8'd0;
            spi_clk_en  <= 1'b0;
        end else begin
            if (spi_clk_cnt >= SPI_CLK_DIV - 1) begin
                spi_clk_cnt <= 8'd0;
                spi_clk_en  <= 1'b1;
            end else begin
                spi_clk_cnt <= spi_clk_cnt + 1'b1;
                spi_clk_en  <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            spi_sclk    <= 1'b0;
            spi_mosi    <= 1'b0;
            spi_bit_cnt <= 4'd0;
            spi_done    <= 1'b0;
            spi_phase   <= 1'b0;
        end else if (spi_active && spi_clk_en) begin
            spi_done <= 1'b0;
            if (!spi_phase) begin
                spi_sclk  <= 1'b0;
                spi_mosi  <= tx_byte[7];
                spi_phase <= 1'b1;
            end else begin
                spi_sclk  <= 1'b1;
                tx_byte   <= {tx_byte[6:0], 1'b0};
                spi_bit_cnt <= spi_bit_cnt + 1'b1;
                spi_phase <= 1'b0;
                if (spi_bit_cnt == 4'd7) begin
                    spi_done    <= 1'b1;
                    spi_bit_cnt <= 4'd0;
                end
            end
        end else if (!spi_active) begin
            spi_sclk  <= 1'b0;
            spi_phase <= 1'b0;
            spi_done  <= 1'b0;
        end
    end

    //======================================================================
    // Display Init Sequence ROM
    // {is_data, delay_flag, byte_value}
    //======================================================================
    localparam NUM_INIT_CMDS = 12;
    reg [9:0] init_rom [0:NUM_INIT_CMDS-1];
    // Format: [9] = needs_delay, [8] = D/C (0=cmd, 1=data), [7:0] = value

    initial begin
        init_rom[0]  = {1'b1, 1'b0, CMD_SWRESET};   // Software reset + delay
        init_rom[1]  = {1'b1, 1'b0, CMD_SLPOUT};     // Sleep out + delay
        init_rom[2]  = {1'b0, 1'b0, CMD_COLMOD};     // Color mode command
        init_rom[3]  = {1'b0, 1'b1, 8'h55};          // 16-bit RGB565
        init_rom[4]  = {1'b0, 1'b0, CMD_MADCTL};     // Memory access control
        init_rom[5]  = {1'b0, 1'b1, 8'h70};          // Landscape, RGB
        init_rom[6]  = {1'b0, 1'b0, CMD_INVON};      // Inversion on (ST7789 needs this)
        init_rom[7]  = {1'b0, 1'b0, CMD_NORON};      // Normal mode
        init_rom[8]  = {1'b0, 1'b0, CMD_GAMSET};     // Gamma set
        init_rom[9]  = {1'b0, 1'b1, 8'h01};          // Gamma curve 1
        init_rom[10] = {1'b1, 1'b0, CMD_DISPON};     // Display on + delay
        init_rom[11] = {1'b0, 1'b0, 8'h00};          // End marker
    end

    //======================================================================
    // Main Display State Machine
    //======================================================================
    localparam [3:0] DS_RESET     = 4'd0,   // Hardware reset
                     DS_RESET_W   = 4'd1,   // Wait after reset
                     DS_INIT      = 4'd2,   // Send init commands
                     DS_INIT_WAIT = 4'd3,   // Delay between init cmds
                     DS_READY     = 4'd4,   // Idle, waiting for refresh
                     DS_SET_COL   = 4'd5,   // Set column address
                     DS_SET_ROW   = 4'd6,   // Set row address
                     DS_WR_START  = 4'd7,   // Start RAM write
                     DS_WR_PIXEL  = 4'd8,   // Write pixel data
                     DS_WR_DONE   = 4'd9;   // Write complete

    reg [3:0]  ds_state;
    reg [31:0] ds_wait;
    reg [3:0]  init_idx;
    reg [23:0] refresh_timer;             // ~10 Hz refresh at 100 MHz

    // Pixel generation state
    reg [7:0]  px_col;                    // Current pixel column
    reg [7:0]  px_row;                    // Current pixel row
    reg [6:0]  char_idx;                  // Character index in buffer
    reg [2:0]  char_col;                  // Column within character (0-7)
    reg [2:0]  char_row;                  // Row within character (0-7)
    reg [7:0]  current_char;              // Current character code
    reg [7:0]  font_byte;                 // Current font row bitmap
    reg [15:0] pixel_color;              // Current pixel color (RGB565)
    reg [1:0]  pixel_byte;               // 0=high byte, 1=low byte

    always @(posedge clk) begin
        if (rst) begin
            ds_state      <= DS_RESET;
            ds_wait       <= 32'd0;
            init_idx      <= 4'd0;
            disp_rst_n    <= 1'b0;
            backlight     <= 1'b0;
            spi_cs_n      <= 1'b1;
            dc            <= 1'b0;
            spi_active    <= 1'b0;
            disp_ready    <= 1'b0;
            disp_busy     <= 1'b0;
            refresh_timer <= 24'd0;
            px_col        <= 8'd0;
            px_row        <= 8'd0;
            pixel_byte    <= 2'd0;
        end else begin
            case (ds_state)
                //----------------------------------------------------------
                // Hardware reset sequence
                DS_RESET: begin
                    disp_rst_n <= 1'b0;
                    spi_cs_n   <= 1'b1;
                    ds_wait    <= 32'd0;
                    ds_state   <= DS_RESET_W;
                end

                DS_RESET_W: begin
                    ds_wait <= ds_wait + 1'b1;
                    if (ds_wait == 32'd1_000_000) begin  // 10ms low
                        disp_rst_n <= 1'b1;
                    end
                    if (ds_wait >= 32'd15_000_000) begin // 150ms total
                        ds_state <= DS_INIT;
                        init_idx <= 4'd0;
                        backlight <= 1'b1;
                    end
                end

                //----------------------------------------------------------
                // Send initialization commands
                DS_INIT: begin
                    if (init_idx >= NUM_INIT_CMDS) begin
                        ds_state   <= DS_READY;
                        disp_ready <= 1'b1;
                        spi_cs_n   <= 1'b1;
                        spi_active <= 1'b0;
                    end else begin
                        spi_cs_n   <= 1'b0;
                        dc         <= init_rom[init_idx][8];  // D/C
                        tx_byte    <= init_rom[init_idx][7:0];
                        spi_active <= 1'b1;
                        ds_state   <= DS_INIT_WAIT;
                    end
                end

                DS_INIT_WAIT: begin
                    if (spi_done) begin
                        spi_active <= 1'b0;
                        spi_cs_n   <= 1'b1;

                        if (init_rom[init_idx][9]) begin
                            // Needs delay
                            ds_wait  <= 32'd0;
                            init_idx <= init_idx + 1'b1;
                            // Wait 120ms for SLPOUT/SWRESET, 20ms for others
                            ds_state <= DS_INIT;
                            // TODO: Add proper delay states
                        end else begin
                            init_idx <= init_idx + 1'b1;
                            ds_state <= DS_INIT;
                        end
                    end
                end

                //----------------------------------------------------------
                // Ready - wait for refresh timer
                DS_READY: begin
                    disp_busy <= 1'b0;
                    refresh_timer <= refresh_timer + 1'b1;

                    // Refresh at ~10 Hz (100M / 10M = 10 Hz)
                    if (refresh_timer >= 24'd10_000_000) begin
                        refresh_timer <= 24'd0;
                        if (char_buf_dirty) begin
                            ds_state  <= DS_SET_COL;
                            disp_busy <= 1'b1;
                            px_col    <= 8'd0;
                            px_row    <= 8'd0;
                        end
                    end
                end

                //----------------------------------------------------------
                // Set column address range (CASET)
                DS_SET_COL: begin
                    // TODO: Send CASET command + 4 data bytes
                    //       (start_col_hi, start_col_lo, end_col_hi, end_col_lo)
                    // For full screen: 0x00, 0x00, 0x00, 0xEF (0 to 239)
                    ds_state <= DS_SET_ROW;
                end

                // Set row address range (RASET)
                DS_SET_ROW: begin
                    // TODO: Send RASET command + 4 data bytes
                    // For 135-pixel height: 0x00, 0x34, 0x00, 0xBB
                    ds_state <= DS_WR_START;
                end

                // Start RAM write (RAMWR)
                DS_WR_START: begin
                    spi_cs_n   <= 1'b0;
                    dc         <= 1'b0;          // Command
                    tx_byte    <= CMD_RAMWR;
                    spi_active <= 1'b1;

                    if (spi_done) begin
                        spi_active <= 1'b0;
                        dc         <= 1'b1;      // Switch to data
                        ds_state   <= DS_WR_PIXEL;
                        pixel_byte <= 2'd0;
                    end
                end

                //----------------------------------------------------------
                // Write pixels (text rendering)
                DS_WR_PIXEL: begin
                    // Determine which character and pixel we're rendering
                    // Character position in buffer
                    char_idx <= (px_row >> 3) * TEXT_COLS + (px_col >> 3);
                    char_col <= px_col[2:0];
                    char_row <= px_row[2:0];

                    // Look up character and font data
                    current_char <= char_buf[char_idx];
                    font_byte    <= font_rom[((current_char - 8'd32) << 3) + char_row];

                    // Determine pixel color (foreground or background)
                    if (font_byte[7 - char_col])
                        pixel_color <= COLOR_WHITE;      // Foreground
                    else
                        pixel_color <= COLOR_BLACK;      // Background

                    // Send pixel data (2 bytes per pixel in RGB565)
                    if (!spi_active) begin
                        if (pixel_byte == 2'd0) begin
                            tx_byte    <= pixel_color[15:8];  // High byte
                            spi_active <= 1'b1;
                            pixel_byte <= 2'd1;
                        end else begin
                            tx_byte    <= pixel_color[7:0];   // Low byte
                            spi_active <= 1'b1;
                            pixel_byte <= 2'd0;

                            // Advance to next pixel
                            if (px_col >= DISP_WIDTH - 1) begin
                                px_col <= 8'd0;
                                if (px_row >= (TEXT_ROWS * CHAR_HEIGHT) - 1) begin
                                    ds_state <= DS_WR_DONE;
                                end else begin
                                    px_row <= px_row + 1'b1;
                                end
                            end else begin
                                px_col <= px_col + 1'b1;
                            end
                        end
                    end

                    if (spi_done)
                        spi_active <= 1'b0;
                end

                //----------------------------------------------------------
                DS_WR_DONE: begin
                    spi_active <= 1'b0;
                    spi_cs_n   <= 1'b1;
                    ds_state   <= DS_READY;
                end

                default: ds_state <= DS_RESET;
            endcase
        end
    end

endmodule
