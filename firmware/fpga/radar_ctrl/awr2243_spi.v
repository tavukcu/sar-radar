//============================================================================
// awr2243_spi.v
// AWR2243 Radar Transceiver SPI Controller
//
// Implements full SPI master for TI AWR2243 configuration and control.
// The AWR2243 uses SPI for all register-level configuration including
// clock setup, RF/analog config, chirp profile, and frame control.
//
// SPI Protocol: Mode 0 (CPOL=0, CPHA=0), MSB first, up to 25 MHz
// AWR2243 SPI frame: 16-bit address + 32-bit data (48-bit total)
//
// Power-up sequence:
//   1. Assert nRESET low, set SOP[2:0] for SPI mode (SOP=100)
//   2. Release nRESET, wait for device ready
//   3. Configure clocks (MasterSS, APLL)
//   4. Configure RF/analog front-end
//   5. Configure chirp profile and frame timing
//   6. Device enters READY state, awaiting frame trigger
//
// Target FPGA: Xilinx Artix-7
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module awr2243_spi #(
    parameter SPI_CLK_DIV     = 2,       // SPI clock = clk / (2 * SPI_CLK_DIV)
    parameter NUM_CONFIG_REGS = 128,     // Number of register writes in config
    parameter RESET_CYCLES    = 250000,  // nRESET hold time (~10ms at 25 MHz)
    parameter POWERUP_WAIT    = 1250000  // Power-up delay (~50ms at 25 MHz)
) (
    input  wire        clk,              // Input clock (25 MHz SPI domain)
    input  wire        rst,              // Synchronous reset

    //------------------------------------------------------------------
    // SPI Physical Interface
    //------------------------------------------------------------------
    output reg         spi_sclk,         // SPI clock
    output reg         spi_mosi,         // Master Out Slave In
    input  wire        spi_miso,         // Master In Slave Out
    output reg         spi_cs_n,         // Chip select (active-low)

    //------------------------------------------------------------------
    // AWR2243 Control Pins
    //------------------------------------------------------------------
    output reg         awr_nreset,       // Device reset (active-low)
    output reg  [2:0]  awr_sop,          // Sense-On-Power mode pins

    //------------------------------------------------------------------
    // Frame Trigger
    //------------------------------------------------------------------
    output reg         frame_trigger,    // Pulse to start a radar frame

    //------------------------------------------------------------------
    // Control Interface
    //------------------------------------------------------------------
    input  wire        start_config,     // Begin configuration sequence
    input  wire        start_frame,      // Begin frame triggering
    output reg         config_done,      // Configuration complete
    output reg         config_error,     // Configuration error occurred
    output reg         frame_active      // Frame is currently in progress
);

    //======================================================================
    // Configuration ROM
    // Stores {address[15:0], data[31:0]} pairs for AWR2243 setup.
    // In production, generate these from TI's mmWave Studio export.
    //======================================================================
    reg [47:0] config_rom [0:NUM_CONFIG_REGS-1];

    // Initialize ROM with AWR2243 configuration sequence
    // These are representative register values - actual values must come
    // from TI mmWave Studio configuration export for your specific chirp profile.
    initial begin
        // ---- Phase 1: Clock Configuration ----
        // MasterSS clock configuration
        config_rom[0]  = {16'h0200, 32'h00000001};  // CLOCK_CTRL: enable internal oscillator
        config_rom[1]  = {16'h0204, 32'h00000006};  // PLL_CTRL: configure APLL
        config_rom[2]  = {16'h0208, 32'h00000040};  // PLL_FREQ: 40 MHz reference

        // ---- Phase 2: RF/Analog Configuration ----
        config_rom[3]  = {16'h0300, 32'h00000077};  // RF_CTRL: enable all 3TX, 4RX
        config_rom[4]  = {16'h0304, 32'h0000001F};  // RX_GAIN: max gain setting
        config_rom[5]  = {16'h0308, 32'h00000003};  // TX_PHASE: TX phase shifter

        // ---- Phase 3: Chirp Profile Configuration ----
        // 77 GHz, 4 GHz bandwidth, ~60us chirp duration
        config_rom[6]  = {16'h0400, 32'h4D400000};  // CHIRP_START_FREQ: 77.0 GHz
        config_rom[7]  = {16'h0404, 32'h06000000};  // CHIRP_FREQ_SLOPE: 66.67 MHz/us
        config_rom[8]  = {16'h0408, 32'h00000200};  // CHIRP_NUM_SAMPLES: 512
        config_rom[9]  = {16'h040C, 32'h000000F0};  // CHIRP_SAMPLE_RATE: 10 Msps
        config_rom[10] = {16'h0410, 32'h00003C00};  // CHIRP_IDLE_TIME: ~10us
        config_rom[11] = {16'h0414, 32'h00000040};  // CHIRP_ADC_START: ~6us

        // ---- Phase 4: Frame Configuration ----
        config_rom[12] = {16'h0500, 32'h00000100};  // FRAME_NUM_CHIRPS: 256 chirps/frame
        config_rom[13] = {16'h0504, 32'h00000001};  // FRAME_NUM_FRAMES: continuous
        config_rom[14] = {16'h0508, 32'h01312D00};  // FRAME_PERIOD: 20ms (50 Hz)
        config_rom[15] = {16'h050C, 32'h00000002};  // FRAME_TRIGGER_MODE: SW trigger

        // ---- Phase 5: LVDS Output Configuration ----
        config_rom[16] = {16'h0600, 32'h00000003};  // LVDS_LANE_ENABLE: 4 data lanes
        config_rom[17] = {16'h0604, 32'h00000001};  // LVDS_DATA_FORMAT: 12-bit ADC
        config_rom[18] = {16'h0608, 32'h00000000};  // LVDS_CLK_CONFIG: SDR mode

        // TODO: Fill remaining config_rom entries from mmWave Studio export
        // Unused entries initialized to NOP (address 0x0000 is safe to write)
    end

    // Initialize remaining ROM entries to zero (NOP writes)
    integer i;
    initial begin
        for (i = 19; i < NUM_CONFIG_REGS; i = i + 1)
            config_rom[i] = 48'd0;
    end

    //======================================================================
    // State Machine
    //======================================================================
    localparam [3:0] S_IDLE        = 4'd0,
                     S_RESET_HOLD  = 4'd1,   // Hold nRESET low
                     S_RESET_WAIT  = 4'd2,   // Wait after reset release
                     S_POWERUP     = 4'd3,   // Wait for device power-up
                     S_CFG_LOAD    = 4'd4,   // Load next config word from ROM
                     S_CFG_SEND    = 4'd5,   // SPI transfer of config word
                     S_CFG_WAIT    = 4'd6,   // Inter-register delay
                     S_CFG_VERIFY  = 4'd7,   // Optional: read-back verify
                     S_READY       = 4'd8,   // Configuration complete
                     S_FRAME_TRIG  = 4'd9,   // Generate frame trigger
                     S_FRAME_WAIT  = 4'd10,  // Wait for frame completion
                     S_ERROR       = 4'd11;

    reg [3:0]  state, state_next;
    reg [31:0] wait_counter;
    reg [7:0]  config_index;              // Current ROM entry index
    reg [47:0] spi_tx_data;               // 48-bit SPI transmit shift register
    reg [47:0] spi_rx_data;               // 48-bit SPI receive shift register
    reg [5:0]  spi_bit_cnt;              // Bit counter for SPI transfer
    reg [7:0]  spi_clk_cnt;              // Clock divider counter
    reg        spi_active;                // SPI transfer in progress

    //======================================================================
    // SPI Clock Divider
    //======================================================================
    reg spi_clk_en;  // Clock enable pulse at SPI rate
    always @(posedge clk) begin
        if (rst || !spi_active) begin
            spi_clk_cnt <= 8'd0;
            spi_clk_en  <= 1'b0;
        end else begin
            if (spi_clk_cnt == SPI_CLK_DIV - 1) begin
                spi_clk_cnt <= 8'd0;
                spi_clk_en  <= 1'b1;
            end else begin
                spi_clk_cnt <= spi_clk_cnt + 1'b1;
                spi_clk_en  <= 1'b0;
            end
        end
    end

    //======================================================================
    // SPI Transfer Engine
    // Mode 0: Data sampled on rising SCLK edge, shifted on falling edge
    //======================================================================
    reg spi_phase;  // 0 = setup phase (falling edge), 1 = sample phase (rising edge)
    reg spi_done;

    always @(posedge clk) begin
        if (rst) begin
            spi_sclk    <= 1'b0;
            spi_mosi    <= 1'b0;
            spi_cs_n    <= 1'b1;
            spi_bit_cnt <= 6'd0;
            spi_phase   <= 1'b0;
            spi_done    <= 1'b0;
            spi_rx_data <= 48'd0;
        end else if (spi_active && spi_clk_en) begin
            spi_done <= 1'b0;
            if (!spi_phase) begin
                // Setup phase (SCLK falling / initial)
                spi_sclk <= 1'b0;
                spi_mosi <= spi_tx_data[47];  // MSB first
                spi_phase <= 1'b1;
            end else begin
                // Sample phase (SCLK rising)
                spi_sclk <= 1'b1;
                spi_rx_data <= {spi_rx_data[46:0], spi_miso};
                spi_tx_data <= {spi_tx_data[46:0], 1'b0};
                spi_bit_cnt <= spi_bit_cnt + 1'b1;
                spi_phase   <= 1'b0;

                if (spi_bit_cnt == 6'd47) begin
                    spi_done    <= 1'b1;
                    spi_bit_cnt <= 6'd0;
                end
            end
        end else if (!spi_active) begin
            spi_sclk    <= 1'b0;
            spi_phase   <= 1'b0;
            spi_bit_cnt <= 6'd0;
            spi_done    <= 1'b0;
        end
    end

    //======================================================================
    // Main State Machine
    //======================================================================
    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            wait_counter   <= 32'd0;
            config_index   <= 8'd0;
            awr_nreset     <= 1'b0;       // Hold in reset
            awr_sop        <= 3'b100;     // SPI mode (SOP2=1, SOP1=0, SOP0=0)
            frame_trigger  <= 1'b0;
            config_done    <= 1'b0;
            config_error   <= 1'b0;
            frame_active   <= 1'b0;
            spi_active     <= 1'b0;
            spi_cs_n       <= 1'b1;
        end else begin
            // Default: clear single-cycle signals
            frame_trigger <= 1'b0;

            case (state)
                //----------------------------------------------------------
                S_IDLE: begin
                    awr_nreset   <= 1'b0;        // Keep in reset
                    awr_sop      <= 3'b100;       // SPI mode
                    config_done  <= 1'b0;
                    config_error <= 1'b0;
                    frame_active <= 1'b0;
                    spi_cs_n     <= 1'b1;
                    if (start_config) begin
                        state        <= S_RESET_HOLD;
                        wait_counter <= 32'd0;
                    end
                end

                //----------------------------------------------------------
                // Hold nRESET low for RESET_CYCLES
                S_RESET_HOLD: begin
                    awr_nreset <= 1'b0;
                    wait_counter <= wait_counter + 1'b1;
                    if (wait_counter >= RESET_CYCLES) begin
                        state        <= S_RESET_WAIT;
                        wait_counter <= 32'd0;
                        awr_nreset   <= 1'b1;    // Release reset
                    end
                end

                //----------------------------------------------------------
                // Wait for device to come out of reset
                S_RESET_WAIT: begin
                    wait_counter <= wait_counter + 1'b1;
                    if (wait_counter >= POWERUP_WAIT) begin
                        state        <= S_CFG_LOAD;
                        wait_counter <= 32'd0;
                        config_index <= 8'd0;
                    end
                end

                //----------------------------------------------------------
                // Load configuration word from ROM
                S_CFG_LOAD: begin
                    if (config_index >= NUM_CONFIG_REGS) begin
                        // All registers written
                        state       <= S_READY;
                        config_done <= 1'b1;
                    end else begin
                        spi_tx_data  <= config_rom[config_index];
                        spi_cs_n     <= 1'b0;     // Assert CS
                        spi_active   <= 1'b1;     // Start SPI transfer
                        state        <= S_CFG_SEND;
                    end
                end

                //----------------------------------------------------------
                // Wait for SPI transfer to complete
                S_CFG_SEND: begin
                    if (spi_done) begin
                        spi_active   <= 1'b0;
                        spi_cs_n     <= 1'b1;     // Deassert CS
                        state        <= S_CFG_WAIT;
                        wait_counter <= 32'd0;
                    end
                end

                //----------------------------------------------------------
                // Inter-register delay (AWR2243 needs time between writes)
                S_CFG_WAIT: begin
                    wait_counter <= wait_counter + 1'b1;
                    if (wait_counter >= 32'd100) begin  // ~4us at 25 MHz
                        config_index <= config_index + 1'b1;
                        state        <= S_CFG_LOAD;
                    end
                end

                //----------------------------------------------------------
                // Configuration complete, device ready for frame triggers
                S_READY: begin
                    config_done <= 1'b1;
                    if (start_frame) begin
                        state        <= S_FRAME_TRIG;
                        frame_active <= 1'b1;
                    end
                end

                //----------------------------------------------------------
                // Generate frame trigger pulse
                S_FRAME_TRIG: begin
                    frame_trigger <= 1'b1;
                    state         <= S_FRAME_WAIT;
                    wait_counter  <= 32'd0;
                end

                //----------------------------------------------------------
                // Wait for frame to complete (frame period)
                S_FRAME_WAIT: begin
                    wait_counter <= wait_counter + 1'b1;
                    // Frame period ~20ms = 500000 cycles at 25 MHz
                    if (wait_counter >= 32'd500000) begin
                        if (start_frame)
                            state <= S_FRAME_TRIG;  // Continuous mode
                        else begin
                            state        <= S_READY;
                            frame_active <= 1'b0;
                        end
                    end
                end

                //----------------------------------------------------------
                S_ERROR: begin
                    config_error <= 1'b1;
                    spi_active   <= 1'b0;
                    spi_cs_n     <= 1'b1;
                    // Stay in error until reset
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
