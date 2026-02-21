#============================================================================
# constraints.xdc
# Xilinx Design Constraints for SAR Radar FPGA
# Target: Artix-7 XC7A100T-1CPG236C (CPG236 package)
#
# IMPORTANT: Pin assignments are PLACEHOLDER values based on the CPG236
# package pinout. Actual pin assignments MUST be updated to match the
# final PCB layout. Verify all assignments against the schematic before
# synthesis.
#
# Bank Allocation Strategy:
#   Bank 14 (VCCO=2.5V): LVDS radar data interface
#   Bank 15 (VCCO=2.5V): LVDS clock inputs
#   Bank 16 (VCCO=3.3V): SPI interfaces (AWR2243, IMU, SD)
#   Bank 34 (VCCO=3.3V): ESP32 SPI, Display, GPIO
#   Bank 35 (VCCO=3.3V): Buttons, LEDs, misc
#
# Author: SAR Radar Project
# Date:   2026-02-21
#============================================================================


#============================================================================
# Clock Constraints
#============================================================================

# 100 MHz system oscillator
# IMPORTANT: Assign to a global clock-capable pin (MRCC or SRCC)
set_property PACKAGE_PIN E3 [get_ports clk_100mhz]
set_property IOSTANDARD LVCMOS33 [get_ports clk_100mhz]
create_clock -period 10.000 -name clk_100mhz [get_ports clk_100mhz]

# PLL generated clocks (auto-derived from MMCM output)
# These are created automatically by the MMCM instantiation,
# but we document them here for reference:
# create_generated_clock -name clk_proc -source [get_pins u_mmcm/CLKIN1] \
#     -multiply_by 2 [get_pins u_mmcm/CLKOUT0]
# create_generated_clock -name clk_spi -source [get_pins u_mmcm/CLKIN1] \
#     -divide_by 4 [get_pins u_mmcm/CLKOUT1]


#============================================================================
# LVDS Radar Data Interface (AWR2243)
# Bank 14/15 - VCCO must be 2.5V for LVDS_25
# Use MRCC/SRCC pairs for clock inputs
#============================================================================

# LVDS bit clock (high-speed, must use MRCC pair)
set_property PACKAGE_PIN D4 [get_ports lvds_clk_p]
set_property PACKAGE_PIN C4 [get_ports lvds_clk_n]
set_property IOSTANDARD LVDS_25 [get_ports lvds_clk_p]
set_property IOSTANDARD LVDS_25 [get_ports lvds_clk_n]
set_property DIFF_TERM TRUE [get_ports lvds_clk_p]
set_property DIFF_TERM TRUE [get_ports lvds_clk_n]

# LVDS frame clock (SRCC pair for regional clock routing)
set_property PACKAGE_PIN D6 [get_ports lvds_frame_p]
set_property PACKAGE_PIN C6 [get_ports lvds_frame_n]
set_property IOSTANDARD LVDS_25 [get_ports lvds_frame_p]
set_property IOSTANDARD LVDS_25 [get_ports lvds_frame_n]
set_property DIFF_TERM TRUE [get_ports lvds_frame_p]
set_property DIFF_TERM TRUE [get_ports lvds_frame_n]

# LVDS data lane 0 (RX channel 0)
set_property PACKAGE_PIN A4 [get_ports {lvds_data_p[0]}]
set_property PACKAGE_PIN A3 [get_ports {lvds_data_n[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_n[0]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_p[0]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_n[0]}]

# LVDS data lane 1 (RX channel 1)
set_property PACKAGE_PIN B6 [get_ports {lvds_data_p[1]}]
set_property PACKAGE_PIN B5 [get_ports {lvds_data_n[1]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_p[1]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_n[1]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_p[1]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_n[1]}]

# LVDS data lane 2 (RX channel 2)
set_property PACKAGE_PIN C7 [get_ports {lvds_data_p[2]}]
set_property PACKAGE_PIN B7 [get_ports {lvds_data_n[2]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_p[2]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_n[2]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_p[2]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_n[2]}]

# LVDS data lane 3 (RX channel 3)
set_property PACKAGE_PIN A6 [get_ports {lvds_data_p[3]}]
set_property PACKAGE_PIN A5 [get_ports {lvds_data_n[3]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_p[3]}]
set_property IOSTANDARD LVDS_25 [get_ports {lvds_data_n[3]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_p[3]}]
set_property DIFF_TERM TRUE [get_ports {lvds_data_n[3]}]

# LVDS bit clock constraint
# AWR2243 LVDS clock frequency depends on configuration
# At 600 Mbps DDR, the clock is 300 MHz
create_clock -period 3.333 -name lvds_bit_clk [get_ports lvds_clk_p]


#============================================================================
# AWR2243 SPI Interface
# Bank 16 - LVCMOS33
#============================================================================

set_property PACKAGE_PIN J1 [get_ports awr_spi_sclk]
set_property PACKAGE_PIN K1 [get_ports awr_spi_mosi]
set_property PACKAGE_PIN L1 [get_ports awr_spi_miso]
set_property PACKAGE_PIN M1 [get_ports awr_spi_cs_n]
set_property PACKAGE_PIN N1 [get_ports awr_nreset]
set_property PACKAGE_PIN P1 [get_ports {awr_sop[0]}]
set_property PACKAGE_PIN R1 [get_ports {awr_sop[1]}]
set_property PACKAGE_PIN T1 [get_ports {awr_sop[2]}]
set_property PACKAGE_PIN J2 [get_ports awr_frame_trig]

set_property IOSTANDARD LVCMOS33 [get_ports awr_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports awr_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports awr_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports awr_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports awr_nreset]
set_property IOSTANDARD LVCMOS33 [get_ports {awr_sop[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports awr_frame_trig]

set_property DRIVE 8 [get_ports awr_spi_sclk]
set_property DRIVE 8 [get_ports awr_spi_mosi]
set_property DRIVE 8 [get_ports awr_spi_cs_n]
set_property SLEW FAST [get_ports awr_spi_sclk]


#============================================================================
# ICM-42688-P IMU SPI Interface
# Bank 16 - LVCMOS33
#============================================================================

set_property PACKAGE_PIN K2 [get_ports imu_spi_sclk]
set_property PACKAGE_PIN L2 [get_ports imu_spi_mosi]
set_property PACKAGE_PIN M2 [get_ports imu_spi_miso]
set_property PACKAGE_PIN N2 [get_ports imu_spi_cs_n]
set_property PACKAGE_PIN P2 [get_ports imu_int1]

set_property IOSTANDARD LVCMOS33 [get_ports imu_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports imu_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports imu_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports imu_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports imu_int1]

set_property DRIVE 8 [get_ports imu_spi_sclk]
set_property SLEW FAST [get_ports imu_spi_sclk]
set_property PULLUP TRUE [get_ports imu_int1]


#============================================================================
# MicroSD Card SPI Interface
# Bank 16 - LVCMOS33
#============================================================================

set_property PACKAGE_PIN R2 [get_ports sd_spi_sclk]
set_property PACKAGE_PIN T2 [get_ports sd_spi_mosi]
set_property PACKAGE_PIN J3 [get_ports sd_spi_miso]
set_property PACKAGE_PIN K3 [get_ports sd_spi_cs_n]

set_property IOSTANDARD LVCMOS33 [get_ports sd_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports sd_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports sd_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports sd_spi_cs_n]

set_property DRIVE 8 [get_ports sd_spi_sclk]
set_property DRIVE 8 [get_ports sd_spi_mosi]
set_property SLEW FAST [get_ports sd_spi_sclk]
set_property PULLUP TRUE [get_ports sd_spi_miso]


#============================================================================
# ESP32-C3 SPI Bridge Interface (FPGA is slave)
# Bank 34 - LVCMOS33
#============================================================================

set_property PACKAGE_PIN L3 [get_ports esp_spi_sclk]
set_property PACKAGE_PIN M3 [get_ports esp_spi_mosi]
set_property PACKAGE_PIN N3 [get_ports esp_spi_miso]
set_property PACKAGE_PIN P3 [get_ports esp_spi_cs_n]

set_property IOSTANDARD LVCMOS33 [get_ports esp_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports esp_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports esp_spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports esp_spi_cs_n]

set_property DRIVE 8 [get_ports esp_spi_miso]
set_property PULLUP TRUE [get_ports esp_spi_cs_n]


#============================================================================
# ST7789 TFT Display SPI Interface
# Bank 34 - LVCMOS33
#============================================================================

set_property PACKAGE_PIN R3 [get_ports disp_spi_sclk]
set_property PACKAGE_PIN T3 [get_ports disp_spi_mosi]
set_property PACKAGE_PIN J4 [get_ports disp_spi_cs_n]
set_property PACKAGE_PIN K4 [get_ports disp_dc]
set_property PACKAGE_PIN L4 [get_ports disp_rst_n]
set_property PACKAGE_PIN M4 [get_ports disp_bl]

set_property IOSTANDARD LVCMOS33 [get_ports disp_spi_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports disp_spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports disp_spi_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports disp_dc]
set_property IOSTANDARD LVCMOS33 [get_ports disp_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports disp_bl]

set_property DRIVE 12 [get_ports disp_spi_sclk]
set_property DRIVE 12 [get_ports disp_spi_mosi]
set_property SLEW FAST [get_ports disp_spi_sclk]
set_property SLEW FAST [get_ports disp_spi_mosi]


#============================================================================
# User Interface - Buttons & LEDs
# Bank 35 - LVCMOS33
#============================================================================

# Buttons (active-low, external pullup)
set_property PACKAGE_PIN N4 [get_ports btn_power]
set_property PACKAGE_PIN P4 [get_ports btn_scan]
set_property PACKAGE_PIN R4 [get_ports btn_mode]

set_property IOSTANDARD LVCMOS33 [get_ports btn_power]
set_property IOSTANDARD LVCMOS33 [get_ports btn_scan]
set_property IOSTANDARD LVCMOS33 [get_ports btn_mode]

set_property PULLUP TRUE [get_ports btn_power]
set_property PULLUP TRUE [get_ports btn_scan]
set_property PULLUP TRUE [get_ports btn_mode]

# LEDs (active-high)
set_property PACKAGE_PIN T4 [get_ports {led_status[0]}]
set_property PACKAGE_PIN J5 [get_ports {led_status[1]}]
set_property PACKAGE_PIN K5 [get_ports {led_status[2]}]
set_property PACKAGE_PIN L5 [get_ports {led_status[3]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led_status[*]}]
set_property DRIVE 8 [get_ports {led_status[*]}]
set_property SLEW SLOW [get_ports {led_status[*]}]

# System reset (active-low, active pullup for safety)
set_property PACKAGE_PIN M5 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP TRUE [get_ports rst_n]


#============================================================================
# Timing Constraints
#============================================================================

# Clock domain crossings - set false paths where appropriate
# LVDS clock domain is async to system clock
set_clock_groups -asynchronous \
    -group [get_clocks clk_100mhz] \
    -group [get_clocks lvds_bit_clk]

# SPI interface timing constraints
# AWR2243 SPI: setup/hold relative to SCLK
set_output_delay -clock clk_100mhz -max 5.0 [get_ports {awr_spi_mosi awr_spi_cs_n}]
set_output_delay -clock clk_100mhz -min 0.0 [get_ports {awr_spi_mosi awr_spi_cs_n}]
set_input_delay  -clock clk_100mhz -max 5.0 [get_ports awr_spi_miso]
set_input_delay  -clock clk_100mhz -min 0.0 [get_ports awr_spi_miso]

# IMU SPI timing
set_output_delay -clock clk_100mhz -max 5.0 [get_ports {imu_spi_mosi imu_spi_cs_n imu_spi_sclk}]
set_output_delay -clock clk_100mhz -min 0.0 [get_ports {imu_spi_mosi imu_spi_cs_n imu_spi_sclk}]
set_input_delay  -clock clk_100mhz -max 5.0 [get_ports imu_spi_miso]
set_input_delay  -clock clk_100mhz -min 0.0 [get_ports imu_spi_miso]

# ESP32 SPI (slave) - inputs are from ESP32 master clock
# These are asynchronous to the FPGA system clock
set_false_path -from [get_ports esp_spi_sclk]
set_false_path -from [get_ports esp_spi_mosi]
set_false_path -from [get_ports esp_spi_cs_n]

# Button inputs are asynchronous
set_false_path -from [get_ports {btn_power btn_scan btn_mode}]

# IMU interrupt is asynchronous
set_false_path -from [get_ports imu_int1]


#============================================================================
# Physical Constraints
#============================================================================

# LVDS data and clock pins should be in the same clock region
# for proper BUFIO/BUFR/ISERDES operation
# set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets lvds_clk_ibuf]

# Bitstream configuration
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]


#============================================================================
# Debug (optional - comment out for production)
#============================================================================
# Uncomment to add ILA debug cores for signal probing
# set_property MARK_DEBUG true [get_nets {u_lvds_capture/sample_valid}]
# set_property MARK_DEBUG true [get_nets {u_lvds_capture/aligned}]
# set_property MARK_DEBUG true [get_nets {state[*]}]
# set_property MARK_DEBUG true [get_nets {frame_counter[*]}]
