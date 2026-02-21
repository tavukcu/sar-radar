//============================================================================
// top_module.v
// Top-level module for 77 GHz Handheld SAR Radar
// Target FPGA: Xilinx Artix-7 XC7A100T-1CPG236C
//
// Architecture Overview:
//   - AWR2243 radar transceiver controlled via SPI
//   - 4-channel LVDS IF data capture at up to 600 Mbps/lane
//   - Real-time 512-point range FFT per chirp
//   - ICM-42688-P IMU for motion compensation
//   - MicroSD logging for raw data capture
//   - ESP32-C3 bridge for WiFi streaming
//   - ST7789 TFT for status display
//
// Clock domains:
//   - clk_sys   : 100 MHz system clock (external oscillator)
//   - clk_proc  : 200 MHz processing clock (PLL generated)
//   - clk_spi   :  25 MHz SPI clock (PLL generated)
//   - clk_lvds  : recovered from AWR2243 LVDS bit clock
//
// Control FSM: IDLE -> CONFIG -> SCANNING -> SAVING -> IDLE
//
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module top_module (
    //------------------------------------------------------------------
    // System
    //------------------------------------------------------------------
    input  wire        clk_100mhz,        // 100 MHz system oscillator
    input  wire        rst_n,             // Active-low external reset

    //------------------------------------------------------------------
    // User Interface
    //------------------------------------------------------------------
    input  wire        btn_power,         // Power button (active-low)
    input  wire        btn_scan,          // Scan start/stop (active-low)
    input  wire        btn_mode,          // Mode select (active-low)
    output wire [3:0]  led_status,        // Status LEDs [power, scan, wifi, error]

    //------------------------------------------------------------------
    // AWR2243 Radar Transceiver
    //------------------------------------------------------------------
    output wire        awr_spi_sclk,     // SPI clock to AWR2243
    output wire        awr_spi_mosi,     // SPI MOSI
    input  wire        awr_spi_miso,     // SPI MISO
    output wire        awr_spi_cs_n,     // SPI chip select (active-low)
    output wire        awr_nreset,       // AWR2243 reset (active-low)
    output wire [2:0]  awr_sop,          // AWR2243 Sense-On-Power pins
    output wire        awr_frame_trig,   // Frame trigger to AWR2243

    //------------------------------------------------------------------
    // AWR2243 LVDS Data Interface
    //------------------------------------------------------------------
    input  wire        lvds_clk_p,       // LVDS bit clock positive
    input  wire        lvds_clk_n,       // LVDS bit clock negative
    input  wire        lvds_frame_p,     // LVDS frame clock positive
    input  wire        lvds_frame_n,     // LVDS frame clock negative
    input  wire [3:0]  lvds_data_p,      // LVDS data lanes positive (4 RX)
    input  wire [3:0]  lvds_data_n,      // LVDS data lanes negative (4 RX)

    //------------------------------------------------------------------
    // ICM-42688-P IMU
    //------------------------------------------------------------------
    output wire        imu_spi_sclk,     // SPI clock to IMU
    output wire        imu_spi_mosi,     // SPI MOSI
    input  wire        imu_spi_miso,     // SPI MISO
    output wire        imu_spi_cs_n,     // SPI chip select (active-low)
    input  wire        imu_int1,         // IMU data-ready interrupt

    //------------------------------------------------------------------
    // MicroSD Card (SPI mode)
    //------------------------------------------------------------------
    output wire        sd_spi_sclk,      // SPI clock to SD card
    output wire        sd_spi_mosi,      // SPI MOSI
    input  wire        sd_spi_miso,      // SPI MISO
    output wire        sd_spi_cs_n,      // SPI chip select (active-low)

    //------------------------------------------------------------------
    // ESP32-C3 SPI Bridge
    //------------------------------------------------------------------
    input  wire        esp_spi_sclk,     // SPI clock from ESP32 (slave)
    input  wire        esp_spi_mosi,     // SPI MOSI from ESP32
    output wire        esp_spi_miso,     // SPI MISO to ESP32
    input  wire        esp_spi_cs_n,     // SPI chip select from ESP32

    //------------------------------------------------------------------
    // ST7789 TFT Display
    //------------------------------------------------------------------
    output wire        disp_spi_sclk,    // SPI clock to display
    output wire        disp_spi_mosi,    // SPI MOSI (data only, no read)
    output wire        disp_spi_cs_n,    // SPI chip select (active-low)
    output wire        disp_dc,          // Data/Command select
    output wire        disp_rst_n,       // Display reset (active-low)
    output wire        disp_bl           // Backlight enable
);

    //======================================================================
    // Parameters
    //======================================================================
    // System state machine
    localparam [2:0] ST_IDLE     = 3'd0,
                     ST_CONFIG   = 3'd1,
                     ST_SCANNING = 3'd2,
                     ST_SAVING   = 3'd3,
                     ST_ERROR    = 3'd4;

    //======================================================================
    // Internal Signals - Clocks & Reset
    //======================================================================
    wire        clk_sys;               // 100 MHz buffered system clock
    wire        clk_proc;              // 200 MHz processing clock
    wire        clk_spi;               //  25 MHz SPI clock
    wire        pll_locked;            // PLL lock indicator
    wire        sys_rst;               // Synchronous active-high reset
    wire        sys_rst_n;             // Synchronous active-low reset

    //======================================================================
    // Internal Signals - Button Debounce
    //======================================================================
    wire        btn_power_db;          // Debounced power button
    wire        btn_scan_db;           // Debounced scan button
    wire        btn_mode_db;           // Debounced mode button
    wire        btn_scan_rising;       // Single-pulse on scan press

    //======================================================================
    // Internal Signals - Radar Control
    //======================================================================
    wire        awr_config_done;       // AWR2243 configuration complete
    wire        awr_config_err;        // AWR2243 configuration error
    wire        awr_frame_active;      // Frame currently active

    //======================================================================
    // Internal Signals - LVDS Capture
    //======================================================================
    wire        lvds_aligned;          // LVDS bitslip alignment done
    wire        lvds_data_valid;       // Captured sample is valid
    wire        lvds_overflow;         // Capture FIFO overflow
    wire [11:0] lvds_ch0_data;         // RX channel 0 ADC sample
    wire [11:0] lvds_ch1_data;         // RX channel 1 ADC sample
    wire [11:0] lvds_ch2_data;         // RX channel 2 ADC sample
    wire [11:0] lvds_ch3_data;         // RX channel 3 ADC sample
    wire        lvds_chirp_start;      // Chirp boundary detected
    wire        lvds_frame_sync;       // Frame sync detected

    //======================================================================
    // Internal Signals - Range FFT
    //======================================================================
    wire        fft_in_ready;          // FFT ready for input
    wire        fft_out_valid;         // FFT output valid
    wire [15:0] fft_out_real;          // FFT output real (I)
    wire [15:0] fft_out_imag;          // FFT output imaginary (Q)
    wire [8:0]  fft_out_index;         // FFT bin index (0-511)
    wire        fft_done;              // FFT complete for one chirp

    //======================================================================
    // Internal Signals - IMU
    //======================================================================
    wire        imu_data_valid;        // IMU data ready
    wire [47:0] imu_timestamp;         // System timestamp
    wire [15:0] imu_accel_x;           // Accelerometer X
    wire [15:0] imu_accel_y;           // Accelerometer Y
    wire [15:0] imu_accel_z;           // Accelerometer Z
    wire [15:0] imu_gyro_x;            // Gyroscope X
    wire [15:0] imu_gyro_y;            // Gyroscope Y
    wire [15:0] imu_gyro_z;            // Gyroscope Z

    //======================================================================
    // Internal Signals - SD Card
    //======================================================================
    wire        sd_ready;              // SD card initialized and ready
    wire        sd_busy;               // SD write in progress
    wire        sd_error;              // SD card error
    wire        sd_write_req;          // Write request to SD module
    wire [7:0]  sd_write_data;         // Byte to write
    wire        sd_write_ack;          // Write acknowledged

    //======================================================================
    // Internal Signals - ESP32 Bridge
    //======================================================================
    wire        esp_buf_empty;         // ESP bridge buffer empty
    wire        esp_buf_full;          // ESP bridge buffer full
    wire [7:0]  esp_status;            // Status byte for ESP32
    wire        esp_data_wr_en;        // Write enable to ESP buffer
    wire [7:0]  esp_data_wr;           // Data byte to ESP buffer

    //======================================================================
    // Internal Signals - Display
    //======================================================================
    wire        disp_ready;            // Display initialized
    wire        disp_busy;             // Display update in progress

    //======================================================================
    // State Machine
    //======================================================================
    reg [2:0]   state, state_next;
    reg [31:0]  frame_counter;         // Total frames captured
    reg [15:0]  chirp_counter;         // Chirps in current frame
    reg         scanning_active;       // Scan is running
    reg [1:0]   mode_select;           // 0=SAR, 1=Range-only, 2=Raw capture

    //======================================================================
    // PLL - Clock Generation
    // Generates 200 MHz processing clock and 25 MHz SPI clock from 100 MHz
    //======================================================================
    // NOTE: In production, replace with Xilinx MMCM/PLL primitive instantiation.
    // Use Vivado Clocking Wizard IP to generate the wrapper.

    // Placeholder: In simulation or when using Clocking Wizard IP
    // clk_wiz_0 u_pll (
    //     .clk_in1    (clk_100mhz),
    //     .clk_out1   (clk_proc),       // 200 MHz
    //     .clk_out2   (clk_spi),        //  25 MHz
    //     .resetn     (rst_n),
    //     .locked     (pll_locked)
    // );

    // For synthesis template - MMCME2_BASE instantiation
    wire clk_100_buf;
    wire clk_proc_unbuf, clk_spi_unbuf;
    wire pll_fb;

    IBUFG u_clk_ibuf (
        .I  (clk_100mhz),
        .O  (clk_100_buf)
    );

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKFBOUT_MULT_F    (10.0),       // VCO = 100 * 10 = 1000 MHz
        .CLKFBOUT_PHASE      (0.0),
        .CLKIN1_PERIOD       (10.0),       // 100 MHz = 10 ns
        .CLKOUT0_DIVIDE_F    (5.0),        // 1000 / 5 = 200 MHz
        .CLKOUT0_PHASE       (0.0),
        .CLKOUT1_DIVIDE      (40),         // 1000 / 40 = 25 MHz
        .CLKOUT1_PHASE       (0.0),
        .CLKOUT2_DIVIDE      (10),         // 1000 / 10 = 100 MHz (sys clock)
        .CLKOUT2_PHASE       (0.0),
        .DIVCLK_DIVIDE       (1),
        .REF_JITTER1         (0.01),
        .STARTUP_WAIT        ("FALSE")
    ) u_mmcm (
        .CLKFBOUT   (pll_fb),
        .CLKFBIN    (pll_fb),
        .CLKIN1     (clk_100_buf),
        .CLKOUT0    (clk_proc_unbuf),
        .CLKOUT1    (clk_spi_unbuf),
        .CLKOUT2    (clk_sys),            // Buffered 100 MHz
        .LOCKED     (pll_locked),
        .PWRDWN     (1'b0),
        .RST        (~rst_n)
    );

    BUFG u_bufg_proc (.I(clk_proc_unbuf), .O(clk_proc));
    BUFG u_bufg_spi  (.I(clk_spi_unbuf),  .O(clk_spi));

    //======================================================================
    // Reset Synchronizer
    // Ensures clean synchronous reset after PLL locks
    //======================================================================
    reg [3:0] rst_sync;
    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n)
            rst_sync <= 4'b0000;
        else if (pll_locked)
            rst_sync <= {rst_sync[2:0], 1'b1};
        else
            rst_sync <= 4'b0000;
    end
    assign sys_rst_n = rst_sync[3];
    assign sys_rst   = ~sys_rst_n;

    //======================================================================
    // Button Debounce (simple shift-register debounce)
    //======================================================================
    button_debounce #(.WIDTH(3)) u_debounce (
        .clk     (clk_sys),
        .rst     (sys_rst),
        .btn_in  ({btn_power, btn_scan, btn_mode}),
        .btn_out ({btn_power_db, btn_scan_db, btn_mode_db})
    );

    // Edge detector for scan button
    reg btn_scan_d;
    always @(posedge clk_sys) begin
        if (sys_rst)
            btn_scan_d <= 1'b1;
        else
            btn_scan_d <= btn_scan_db;
    end
    assign btn_scan_rising = btn_scan_d & ~btn_scan_db;  // Active-low press

    //======================================================================
    // Submodule Instantiations
    //======================================================================

    //----------------------------------------------------------------------
    // AWR2243 SPI Controller
    //----------------------------------------------------------------------
    awr2243_spi #(
        .SPI_CLK_DIV    (2),               // clk_spi/2 = 12.5 MHz SPI
        .NUM_CONFIG_REGS (128)              // Number of config register writes
    ) u_awr2243_spi (
        .clk            (clk_spi),
        .rst            (sys_rst),
        // SPI physical interface
        .spi_sclk       (awr_spi_sclk),
        .spi_mosi       (awr_spi_mosi),
        .spi_miso       (awr_spi_miso),
        .spi_cs_n       (awr_spi_cs_n),
        // AWR2243 control
        .awr_nreset     (awr_nreset),
        .awr_sop        (awr_sop),
        .frame_trigger   (awr_frame_trig),
        // Control interface
        .start_config   (state == ST_CONFIG),
        .start_frame    (scanning_active),
        .config_done    (awr_config_done),
        .config_error   (awr_config_err),
        .frame_active   (awr_frame_active)
    );

    //----------------------------------------------------------------------
    // LVDS Data Capture
    //----------------------------------------------------------------------
    lvds_capture u_lvds_capture (
        .clk_sys        (clk_sys),
        .clk_proc       (clk_proc),
        .rst            (sys_rst),
        // LVDS physical interface
        .lvds_clk_p     (lvds_clk_p),
        .lvds_clk_n     (lvds_clk_n),
        .lvds_frame_p   (lvds_frame_p),
        .lvds_frame_n   (lvds_frame_n),
        .lvds_data_p    (lvds_data_p),
        .lvds_data_n    (lvds_data_n),
        // Control
        .enable         (state == ST_SCANNING),
        .aligned        (lvds_aligned),
        // Data output (clk_proc domain)
        .data_valid     (lvds_data_valid),
        .ch0_data       (lvds_ch0_data),
        .ch1_data       (lvds_ch1_data),
        .ch2_data       (lvds_ch2_data),
        .ch3_data       (lvds_ch3_data),
        .chirp_start    (lvds_chirp_start),
        .frame_sync     (lvds_frame_sync),
        .overflow       (lvds_overflow)
    );

    //----------------------------------------------------------------------
    // Range FFT Processor (processes channel 0; replicate for other channels)
    //----------------------------------------------------------------------
    range_fft #(
        .FFT_SIZE       (512),
        .INPUT_WIDTH    (12),
        .OUTPUT_WIDTH   (16)
    ) u_range_fft (
        .clk            (clk_proc),
        .rst            (sys_rst),
        // Input
        .in_valid       (lvds_data_valid),
        .in_data        (lvds_ch0_data),
        .in_ready       (fft_in_ready),
        .chirp_start    (lvds_chirp_start),
        // Output
        .out_valid      (fft_out_valid),
        .out_real       (fft_out_real),
        .out_imag       (fft_out_imag),
        .out_index      (fft_out_index),
        .fft_done       (fft_done)
    );

    //----------------------------------------------------------------------
    // ICM-42688-P IMU Reader
    //----------------------------------------------------------------------
    imu_spi #(
        .SPI_CLK_DIV    (2),               // ~12.5 MHz SPI
        .ODR_DIVIDER    (100000)            // 100 MHz / 100000 = 1 kHz ODR
    ) u_imu_spi (
        .clk            (clk_sys),
        .rst            (sys_rst),
        // SPI physical interface
        .spi_sclk       (imu_spi_sclk),
        .spi_mosi       (imu_spi_mosi),
        .spi_miso       (imu_spi_miso),
        .spi_cs_n       (imu_spi_cs_n),
        // IMU interrupt
        .data_ready_int (imu_int1),
        // Data output
        .data_valid     (imu_data_valid),
        .timestamp      (imu_timestamp),
        .accel_x        (imu_accel_x),
        .accel_y        (imu_accel_y),
        .accel_z        (imu_accel_z),
        .gyro_x         (imu_gyro_x),
        .gyro_y         (imu_gyro_y),
        .gyro_z         (imu_gyro_z)
    );

    //----------------------------------------------------------------------
    // MicroSD Card Writer
    //----------------------------------------------------------------------
    sd_spi u_sd_spi (
        .clk            (clk_sys),
        .rst            (sys_rst),
        // SPI physical interface
        .spi_sclk       (sd_spi_sclk),
        .spi_mosi       (sd_spi_mosi),
        .spi_miso       (sd_spi_miso),
        .spi_cs_n       (sd_spi_cs_n),
        // Control
        .sd_ready       (sd_ready),
        .sd_busy        (sd_busy),
        .sd_error       (sd_error),
        // Write interface
        .write_req      (sd_write_req),
        .write_data     (sd_write_data),
        .write_ack      (sd_write_ack)
    );

    //----------------------------------------------------------------------
    // ESP32-C3 SPI Bridge (FPGA is slave)
    //----------------------------------------------------------------------
    spi_bridge u_spi_bridge (
        .clk            (clk_sys),
        .rst            (sys_rst),
        // SPI slave interface
        .spi_sclk       (esp_spi_sclk),
        .spi_mosi       (esp_spi_mosi),
        .spi_miso       (esp_spi_miso),
        .spi_cs_n       (esp_spi_cs_n),
        // Data input (from FFT / capture pipeline)
        .data_wr_en     (esp_data_wr_en),
        .data_wr        (esp_data_wr),
        // Status
        .status_byte    (esp_status),
        .buf_empty      (esp_buf_empty),
        .buf_full       (esp_buf_full)
    );

    //----------------------------------------------------------------------
    // ST7789 Display Controller
    //----------------------------------------------------------------------
    display_spi u_display_spi (
        .clk            (clk_sys),
        .rst            (sys_rst),
        // SPI physical interface
        .spi_sclk       (disp_spi_sclk),
        .spi_mosi       (disp_spi_mosi),
        .spi_cs_n       (disp_spi_cs_n),
        .dc             (disp_dc),
        .disp_rst_n     (disp_rst_n),
        .backlight      (disp_bl),
        // Status inputs for display
        .scan_active    (scanning_active),
        .frame_count    (frame_counter),
        .wifi_connected (~esp_buf_empty),   // Simplified indicator
        .sd_status      ({sd_ready, sd_busy, sd_error}),
        .system_state   (state),
        .disp_ready     (disp_ready),
        .disp_busy      (disp_busy)
    );

    //======================================================================
    // Main Control State Machine
    //======================================================================
    always @(posedge clk_sys) begin
        if (sys_rst) begin
            state           <= ST_IDLE;
            frame_counter   <= 32'd0;
            chirp_counter   <= 16'd0;
            scanning_active <= 1'b0;
            mode_select     <= 2'd0;
        end else begin
            state <= state_next;

            // Frame counter
            if (state == ST_SCANNING && lvds_frame_sync)
                frame_counter <= frame_counter + 1'b1;

            // Chirp counter (resets each frame)
            if (lvds_frame_sync)
                chirp_counter <= 16'd0;
            else if (lvds_chirp_start)
                chirp_counter <= chirp_counter + 1'b1;

            // Scan control via button toggle
            if (btn_scan_rising) begin
                if (state == ST_IDLE || state == ST_SAVING)
                    scanning_active <= 1'b1;
                else if (state == ST_SCANNING)
                    scanning_active <= 1'b0;
            end

            // Mode cycling
            if (btn_mode_db == 1'b0 && state == ST_IDLE)
                mode_select <= mode_select + 1'b1;
        end
    end

    // Next-state logic
    always @(*) begin
        state_next = state;
        case (state)
            ST_IDLE: begin
                if (scanning_active)
                    state_next = ST_CONFIG;
            end

            ST_CONFIG: begin
                if (awr_config_done && sd_ready)
                    state_next = ST_SCANNING;
                else if (awr_config_err)
                    state_next = ST_ERROR;
            end

            ST_SCANNING: begin
                if (!scanning_active)
                    state_next = ST_SAVING;
                else if (lvds_overflow)
                    state_next = ST_ERROR;
            end

            ST_SAVING: begin
                // Wait for SD writes to complete
                if (!sd_busy)
                    state_next = ST_IDLE;
            end

            ST_ERROR: begin
                // Return to idle after button press
                if (btn_scan_rising)
                    state_next = ST_IDLE;
            end

            default: state_next = ST_IDLE;
        endcase
    end

    //======================================================================
    // Data Routing - FFT output to ESP32 bridge and SD writer
    //======================================================================
    // Pack FFT output for ESP32 streaming
    assign esp_data_wr_en = fft_out_valid & (state == ST_SCANNING);
    assign esp_data_wr    = fft_out_real[15:8]; // TODO: implement proper serialization

    // SD write request when FFT or IMU data available
    assign sd_write_req  = (fft_out_valid | imu_data_valid) & (state == ST_SCANNING);
    assign sd_write_data = fft_out_valid ? fft_out_real[15:8] : imu_accel_x[15:8];
    // TODO: Implement proper data packer that serializes full FFT frames
    //       and IMU data packets into 512-byte SD blocks

    //======================================================================
    // ESP32 Status Byte
    //======================================================================
    assign esp_status = {
        state[2:0],        // [7:5] system state
        scanning_active,   // [4]   scan active
        lvds_aligned,      // [3]   LVDS aligned
        sd_ready,          // [2]   SD ready
        lvds_overflow,     // [1]   overflow error
        awr_config_err     // [0]   config error
    };

    //======================================================================
    // LED Status Indicators
    //======================================================================
    assign led_status[0] = pll_locked;                      // Power/ready
    assign led_status[1] = scanning_active;                 // Scanning
    assign led_status[2] = ~esp_buf_empty;                  // WiFi activity
    assign led_status[3] = (state == ST_ERROR) | sd_error;  // Error

endmodule


//==========================================================================
// Button Debounce Module
// Simple shift-register debouncer for multiple buttons
//==========================================================================
module button_debounce #(
    parameter WIDTH = 3,
    parameter DEBOUNCE_BITS = 20    // ~10ms at 100 MHz
) (
    input  wire              clk,
    input  wire              rst,
    input  wire [WIDTH-1:0]  btn_in,
    output reg  [WIDTH-1:0]  btn_out
);
    reg [DEBOUNCE_BITS-1:0] counter;
    reg [WIDTH-1:0]         btn_sync0, btn_sync1;
    wire                    sample_tick;

    // Free-running counter for sample tick
    always @(posedge clk) begin
        if (rst)
            counter <= {DEBOUNCE_BITS{1'b0}};
        else
            counter <= counter + 1'b1;
    end
    assign sample_tick = (counter == {DEBOUNCE_BITS{1'b0}});

    // Double-sync and sample
    always @(posedge clk) begin
        if (rst) begin
            btn_sync0 <= {WIDTH{1'b1}};
            btn_sync1 <= {WIDTH{1'b1}};
            btn_out   <= {WIDTH{1'b1}};
        end else begin
            btn_sync0 <= btn_in;
            btn_sync1 <= btn_sync0;
            if (sample_tick)
                btn_out <= btn_sync1;
        end
    end
endmodule
