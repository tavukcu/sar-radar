//============================================================================
// imu_spi.v
// ICM-42688-P IMU SPI Reader
//
// Reads 6-axis inertial data (3-axis accelerometer + 3-axis gyroscope)
// from the TDK InvenSense ICM-42688-P MEMS IMU via SPI. The IMU data
// is critical for SAR image reconstruction - it provides the antenna
// position/motion information needed for coherent aperture synthesis.
//
// ICM-42688-P SPI Specifications:
//   - SPI Mode 3 (CPOL=1, CPHA=1)
//   - Max SPI clock: 24 MHz (read), 1 MHz (register write at startup)
//   - Register read: [1:R/W | 7:ADDR] + [8:DATA_OUT]
//   - Burst read: address auto-increments
//
// Data Output Format (160 bits total):
//   {timestamp[47:0], ax[15:0], ay[15:0], az[15:0],
//    gx[15:0], gy[15:0], gz[15:0]}
//
// The module handles:
//   1. Device initialization on reset
//   2. Continuous sensor data reading at configurable ODR
//   3. Timestamping each measurement with system clock counter
//
// Target FPGA: Xilinx Artix-7
// Author: SAR Radar Project
// Date:   2026-02-21
//============================================================================

`timescale 1ns / 1ps

module imu_spi #(
    parameter SPI_CLK_DIV  = 2,          // SPI clock = clk / (2 * SPI_CLK_DIV)
    parameter ODR_DIVIDER  = 100000      // Readout interval in clk cycles (1 kHz at 100 MHz)
) (
    input  wire        clk,              // System clock (100 MHz)
    input  wire        rst,              // Synchronous reset

    //------------------------------------------------------------------
    // SPI Physical Interface
    //------------------------------------------------------------------
    output reg         spi_sclk,         // SPI clock (idle high, Mode 3)
    output reg         spi_mosi,         // Master Out Slave In
    input  wire        spi_miso,         // Master In Slave Out
    output reg         spi_cs_n,         // Chip select (active-low)

    //------------------------------------------------------------------
    // IMU Interrupt
    //------------------------------------------------------------------
    input  wire        data_ready_int,   // INT1 from ICM-42688-P (active-high)

    //------------------------------------------------------------------
    // Data Output
    //------------------------------------------------------------------
    output reg         data_valid,       // Output data valid pulse
    output reg  [47:0] timestamp,        // System timestamp (clk cycles)
    output reg  [15:0] accel_x,          // Accelerometer X (signed, mg)
    output reg  [15:0] accel_y,          // Accelerometer Y
    output reg  [15:0] accel_z,          // Accelerometer Z
    output reg  [15:0] gyro_x,           // Gyroscope X (signed, dps)
    output reg  [15:0] gyro_y,           // Gyroscope Y
    output reg  [15:0] gyro_z            // Gyroscope Z
);

    //======================================================================
    // ICM-42688-P Register Map (relevant subset)
    //======================================================================
    localparam REG_WHO_AM_I      = 8'h75;  // Expected value: 0x47
    localparam REG_PWR_MGMT0     = 8'h4E;  // Power management
    localparam REG_GYRO_CONFIG0  = 8'h4F;  // Gyro range & ODR
    localparam REG_ACCEL_CONFIG0 = 8'h50;  // Accel range & ODR
    localparam REG_GYRO_CONFIG1  = 8'h51;  // Gyro filter config
    localparam REG_ACCEL_CONFIG1 = 8'h53;  // Accel filter config
    localparam REG_INT_CONFIG    = 8'h14;  // Interrupt config
    localparam REG_INT_SOURCE0   = 8'h65;  // INT1 source selection
    localparam REG_TEMP_DATA1    = 8'h1D;  // Temperature data MSB
    localparam REG_ACCEL_DATA_X1 = 8'h1F;  // Accel X MSB (burst read start)
    // Burst read order: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
    //                   GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L

    // Configuration values
    localparam CFG_PWR_MGMT0     = 8'h0F;  // Accel + Gyro in Low-Noise mode
    localparam CFG_GYRO_CONFIG0  = 8'h06;  // +/-2000 dps, 1 kHz ODR
    localparam CFG_ACCEL_CONFIG0 = 8'h06;  // +/-16g, 1 kHz ODR
    localparam CFG_INT_CONFIG    = 8'h02;  // INT1 push-pull, active-high
    localparam CFG_INT_SOURCE0   = 8'h08;  // Data-ready routed to INT1

    localparam EXPECTED_WHO_AM_I = 8'h47;  // ICM-42688-P device ID

    //======================================================================
    // Initialization Sequence ROM
    // Format: {R/W, 7'addr, 8'data}
    // R/W = 0 for write, 1 for read
    //======================================================================
    localparam NUM_INIT_CMDS = 7;
    reg [15:0] init_rom [0:NUM_INIT_CMDS-1];

    initial begin
        // Step 0: Read WHO_AM_I to verify device
        init_rom[0] = {1'b1, REG_WHO_AM_I[6:0], 8'h00};
        // Step 1: Configure gyroscope (range + ODR)
        init_rom[1] = {1'b0, REG_GYRO_CONFIG0[6:0], CFG_GYRO_CONFIG0};
        // Step 2: Configure accelerometer (range + ODR)
        init_rom[2] = {1'b0, REG_ACCEL_CONFIG0[6:0], CFG_ACCEL_CONFIG0};
        // Step 3: Configure interrupt pin
        init_rom[3] = {1'b0, REG_INT_CONFIG[6:0], CFG_INT_CONFIG};
        // Step 4: Route data-ready to INT1
        init_rom[4] = {1'b0, REG_INT_SOURCE0[6:0], CFG_INT_SOURCE0};
        // Step 5: Power on sensors (must be last config write)
        init_rom[5] = {1'b0, REG_PWR_MGMT0[6:0], CFG_PWR_MGMT0};
        // Step 6: Dummy read (wait for sensors to start)
        init_rom[6] = {1'b1, REG_WHO_AM_I[6:0], 8'h00};
    end

    //======================================================================
    // SPI Engine
    //======================================================================
    // SPI Mode 3: CPOL=1 (idle high), CPHA=1 (sample on falling edge)
    reg [7:0]  spi_clk_cnt;
    reg        spi_clk_en;               // Clock enable at SPI rate
    reg [15:0] spi_tx_shift;             // Transmit shift register
    reg [7:0]  spi_rx_shift;             // Receive shift register
    reg [4:0]  spi_bit_cnt;              // Bit counter
    reg        spi_active;               // Transfer in progress
    reg        spi_done;                 // Transfer complete pulse
    reg        spi_phase;                // Clock phase tracking
    reg [7:0]  spi_rx_byte;              // Received byte

    // SPI clock divider
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

    // SPI shift engine (Mode 3: CPOL=1, CPHA=1)
    always @(posedge clk) begin
        if (rst) begin
            spi_sclk      <= 1'b1;       // Idle high (CPOL=1)
            spi_mosi      <= 1'b0;
            spi_bit_cnt   <= 5'd0;
            spi_phase     <= 1'b0;
            spi_done      <= 1'b0;
            spi_rx_shift  <= 8'd0;
            spi_rx_byte   <= 8'd0;
        end else if (spi_active && spi_clk_en) begin
            spi_done <= 1'b0;
            if (!spi_phase) begin
                // Rising edge: shift out data (CPHA=1: data changes on rising)
                spi_sclk <= 1'b1;
                spi_mosi <= spi_tx_shift[15];
                spi_tx_shift <= {spi_tx_shift[14:0], 1'b0};
                spi_phase <= 1'b1;
            end else begin
                // Falling edge: sample data
                spi_sclk <= 1'b0;
                spi_rx_shift <= {spi_rx_shift[6:0], spi_miso};
                spi_bit_cnt <= spi_bit_cnt + 1'b1;
                spi_phase <= 1'b0;

                if (spi_bit_cnt == 5'd15) begin
                    spi_done    <= 1'b1;
                    spi_rx_byte <= {spi_rx_shift[6:0], spi_miso};
                    spi_bit_cnt <= 5'd0;
                end
            end
        end else if (!spi_active) begin
            spi_sclk    <= 1'b1;          // Idle high
            spi_phase   <= 1'b0;
            spi_bit_cnt <= 5'd0;
            spi_done    <= 1'b0;
        end
    end

    //======================================================================
    // System Timestamp Counter
    // Free-running 48-bit counter at system clock rate
    // At 100 MHz, wraps every ~2.8 million seconds (~32 days)
    //======================================================================
    reg [47:0] sys_timestamp;
    always @(posedge clk) begin
        if (rst)
            sys_timestamp <= 48'd0;
        else
            sys_timestamp <= sys_timestamp + 1'b1;
    end

    //======================================================================
    // Main State Machine
    //======================================================================
    localparam [3:0] S_IDLE        = 4'd0,
                     S_INIT_START  = 4'd1,   // Begin init sequence
                     S_INIT_CS     = 4'd2,   // Assert CS for init write/read
                     S_INIT_XFER   = 4'd3,   // SPI transfer for init
                     S_INIT_DONE   = 4'd4,   // Deassert CS, check result
                     S_INIT_DELAY  = 4'd5,   // Inter-command delay
                     S_WAIT_DRDY   = 4'd6,   // Wait for data-ready interrupt
                     S_READ_START  = 4'd7,   // Start burst read
                     S_READ_ADDR   = 4'd8,   // Send read address
                     S_READ_BURST  = 4'd9,   // Burst read 12 bytes
                     S_READ_DONE   = 4'd10,  // Process and output data
                     S_ERROR       = 4'd11;

    reg [3:0]  state;
    reg [3:0]  init_index;                // Current init ROM index
    reg [31:0] delay_counter;             // General-purpose delay counter
    reg [3:0]  burst_byte_cnt;            // Byte counter for burst read
    reg [7:0]  burst_data [0:11];         // Storage for 12 burst bytes
    reg        init_done;                 // Initialization complete
    reg        drdy_sync0, drdy_sync1;    // Data-ready synchronizer
    wire       drdy_rising;

    // Synchronize data-ready interrupt
    always @(posedge clk) begin
        if (rst) begin
            drdy_sync0 <= 1'b0;
            drdy_sync1 <= 1'b0;
        end else begin
            drdy_sync0 <= data_ready_int;
            drdy_sync1 <= drdy_sync0;
        end
    end
    assign drdy_rising = drdy_sync0 & ~drdy_sync1;

    // Main FSM
    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            init_index     <= 4'd0;
            delay_counter  <= 32'd0;
            burst_byte_cnt <= 4'd0;
            init_done      <= 1'b0;
            spi_active     <= 1'b0;
            spi_cs_n       <= 1'b1;
            data_valid     <= 1'b0;
            timestamp      <= 48'd0;
            accel_x        <= 16'd0;
            accel_y        <= 16'd0;
            accel_z        <= 16'd0;
            gyro_x         <= 16'd0;
            gyro_y         <= 16'd0;
            gyro_z         <= 16'd0;
        end else begin
            // Defaults
            data_valid <= 1'b0;

            case (state)
                //----------------------------------------------------------
                S_IDLE: begin
                    // Start initialization after power-up delay
                    delay_counter <= delay_counter + 1'b1;
                    if (delay_counter >= 32'd10_000_000) begin // 100ms at 100 MHz
                        state         <= S_INIT_START;
                        init_index    <= 4'd0;
                        delay_counter <= 32'd0;
                    end
                end

                //----------------------------------------------------------
                // Initialization Sequence
                S_INIT_START: begin
                    if (init_index >= NUM_INIT_CMDS) begin
                        init_done <= 1'b1;
                        state     <= S_WAIT_DRDY;
                    end else begin
                        // Load command from ROM
                        spi_tx_shift <= init_rom[init_index];
                        state        <= S_INIT_CS;
                    end
                end

                S_INIT_CS: begin
                    spi_cs_n   <= 1'b0;           // Assert CS
                    spi_active <= 1'b1;
                    state      <= S_INIT_XFER;
                end

                S_INIT_XFER: begin
                    if (spi_done) begin
                        spi_active <= 1'b0;
                        spi_cs_n   <= 1'b1;       // Deassert CS
                        state      <= S_INIT_DONE;
                    end
                end

                S_INIT_DONE: begin
                    // Check WHO_AM_I response (init_index 0)
                    if (init_index == 4'd0) begin
                        if (spi_rx_byte != EXPECTED_WHO_AM_I) begin
                            state <= S_ERROR;       // Wrong device ID
                        end else begin
                            init_index    <= init_index + 1'b1;
                            delay_counter <= 32'd0;
                            state         <= S_INIT_DELAY;
                        end
                    end else begin
                        init_index    <= init_index + 1'b1;
                        delay_counter <= 32'd0;
                        state         <= S_INIT_DELAY;
                    end
                end

                S_INIT_DELAY: begin
                    // Wait between register writes (~1ms)
                    delay_counter <= delay_counter + 1'b1;
                    if (delay_counter >= 32'd100_000) begin // 1ms at 100 MHz
                        state <= S_INIT_START;
                    end
                end

                //----------------------------------------------------------
                // Normal Operation: Wait for Data Ready
                S_WAIT_DRDY: begin
                    if (drdy_rising) begin
                        timestamp      <= sys_timestamp;  // Capture timestamp
                        state          <= S_READ_START;
                        burst_byte_cnt <= 4'd0;
                    end
                end

                //----------------------------------------------------------
                // Burst Read: Read 12 bytes starting from ACCEL_DATA_X1
                S_READ_START: begin
                    // Send read command: bit7=1 (read), addr = ACCEL_DATA_X1
                    spi_tx_shift <= {1'b1, REG_ACCEL_DATA_X1[6:0], 8'hFF};
                    spi_cs_n     <= 1'b0;
                    spi_active   <= 1'b1;
                    state        <= S_READ_ADDR;
                end

                S_READ_ADDR: begin
                    if (spi_done) begin
                        // First byte received during address phase (discard)
                        // Now burst-read 12 bytes
                        burst_byte_cnt <= 4'd0;
                        spi_tx_shift   <= {16'hFFFF};  // Clock out dummy bytes
                        spi_active     <= 1'b1;
                        state          <= S_READ_BURST;
                    end
                end

                S_READ_BURST: begin
                    if (spi_done) begin
                        burst_data[burst_byte_cnt] <= spi_rx_byte;
                        burst_byte_cnt <= burst_byte_cnt + 1'b1;

                        if (burst_byte_cnt >= 4'd11) begin
                            // All 12 bytes received
                            spi_active <= 1'b0;
                            spi_cs_n   <= 1'b1;
                            state      <= S_READ_DONE;
                        end else begin
                            // Continue burst - send more dummy bytes
                            spi_tx_shift <= {16'hFFFF};
                            spi_active   <= 1'b1;
                        end
                    end
                end

                S_READ_DONE: begin
                    // Assemble 16-bit values from burst data
                    // Byte order: AX_H[0], AX_L[1], AY_H[2], AY_L[3],
                    //             AZ_H[4], AZ_L[5], GX_H[6], GX_L[7],
                    //             GY_H[8], GY_L[9], GZ_H[10], GZ_L[11]
                    accel_x <= {burst_data[0],  burst_data[1]};
                    accel_y <= {burst_data[2],  burst_data[3]};
                    accel_z <= {burst_data[4],  burst_data[5]};
                    gyro_x  <= {burst_data[6],  burst_data[7]};
                    gyro_y  <= {burst_data[8],  burst_data[9]};
                    gyro_z  <= {burst_data[10], burst_data[11]};
                    data_valid <= 1'b1;
                    state      <= S_WAIT_DRDY;
                end

                //----------------------------------------------------------
                S_ERROR: begin
                    // IMU communication error - stay here until reset
                    spi_active <= 1'b0;
                    spi_cs_n   <= 1'b1;
                    // TODO: Implement retry logic or error reporting
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
