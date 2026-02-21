# Main Board - SAR Radar Digital & Power Platform

## Overview

The main board serves as the central processing, control, and power management platform for the handheld SAR radar. It hosts the FPGA for real-time signal processing, IMU for motion compensation, wireless connectivity, user interface, data storage, and a complete USB-C PD charging system. It connects to the RF module via a board-to-board connector.

## Key Components

### Xilinx Artix-7 XC7A100T-1CPG236C FPGA
- 101,440 logic cells, 4,860 Kbit block RAM, 240 DSP48E1 slices
- Package: CPG236 (BGA, 236 balls, 10 mm x 10 mm)
- Speed grade: -1 (commercial)
- Functions:
  - LVDS deserializer for AWR2243 ADC data
  - Range-Doppler FFT processing (2D FFT pipeline)
  - Raw data buffering and DMA to microSD
  - SPI master for AWR2243 configuration
  - SPI master for IMU readout
  - Display controller (SPI to ST7789)
- Configuration: SPI flash (W25Q128JVSIQ), JTAG header for debug
- Clock: 100 MHz primary oscillator, 200 MHz derived via MMCM

### ICM-42688-P 6-DOF IMU
- 3-axis accelerometer + 3-axis gyroscope
- Package: LGA-14 (2.5 x 3.0 mm)
- Interface: SPI (up to 24 MHz), connected to FPGA
- Output data rate: configurable up to 32 kHz
- Accelerometer range: +/-16g
- Gyroscope range: +/-2000 dps
- Purpose: motion tracking for SAR autofocus and motion compensation algorithms
- Placement: as close to antenna phase center as feasible, aligned with board axes

### ESP32-C3-MINI-1 WiFi/BT Module
- RISC-V single-core, 160 MHz
- WiFi 4 (802.11 b/g/n) + Bluetooth 5.0 LE
- Interface: UART to FPGA (data transfer), SPI (firmware update)
- Functions:
  - Stream processed SAR images to smartphone/tablet via WiFi
  - BLE beacon for device discovery
  - OTA firmware update capability
- Onboard PCB antenna, keep-out zone per datasheet (10 mm clearance)

### ST7789 1.3" TFT Display
- Resolution: 240 x 240 pixels
- Interface: SPI (4-wire), directly driven by FPGA
- Connector: 13-pin FPC (0.5 mm pitch)
- Purpose: real-time range-Doppler map display, battery status, system status
- Backlight: white LED, PWM brightness control from FPGA GPIO

### microSD Card Slot
- Interface: SPI mode (primary) / SDIO 4-bit (optional via FPGA)
- Purpose: raw ADC data logging, SAR image storage
- Card detect switch included
- Recommended card: UHS-I, Class 10 minimum for sustained write >10 MB/s

### USB-C Power Delivery Charging System

#### BQ25792RWWR Battery Charger
- Input: USB-C, 5-20V PD negotiated
- Battery: 2S Li-Po (7.4V nominal, 8.4V full)
- Charge current: up to 3A (configurable via I2C)
- Features: MPPT, input current limit, thermal regulation, NTC monitoring
- I2C interface to ESP32-C3 for charge status reporting

#### TPS25750DRGR USB-C PD Controller
- USB Power Delivery 3.0 compliant
- Negotiates 9V/2A or 12V/1.5A from PD source
- CC logic, VBUS management
- I2C configuration interface
- Handles cable orientation detection and role swap

### Power Regulators

| Regulator        | Part Number      | Topology | Output | Load              |
|------------------|------------------|----------|--------|-------------------|
| TPS62827DMQR #1  | TPS62827DMQR     | Buck     | 3.3V   | FPGA I/O, ESP32, SD, misc |
| TPS62827DMQR #2  | TPS62827DMQR     | Buck     | 1.8V   | FPGA aux, LVDS    |
| TPS62827DMQR #3  | TPS62827DMQR     | Buck     | 1.0V   | FPGA core (VCCINT)|
| TPS62827DMQR #4  | TPS62827DMQR     | Buck     | 0.9V   | FPGA BRAM (VCCBRAM)|
| TPS7A4700RGWR    | TPS7A4700RGWR    | LDO      | 1.8V   | RF analog (via connector to RF board) |

- All buck converters operate from battery voltage (6.0-8.4V).
- TPS62827: 2 MHz switching, 2A output, DCS-Control for fast transient response.
- Sequencing: 3.3V first, then 1.8V, then 1.0V/0.9V (FPGA power-up sequence per UG475).
- Power-good signals from each regulator daisy-chained for sequencing.

## Board Specifications

| Parameter         | Value                          |
|-------------------|--------------------------------|
| Substrate         | FR4 (Tg >= 170C)              |
| Layer count       | 6                              |
| Board dimensions  | 80 x 60 mm                     |
| Copper weight     | 1 oz (all layers)              |
| Surface finish    | ENIG                           |
| Min trace/space   | 3.5/3.5 mil                    |
| Min via           | 0.2 mm drill / 0.45 mm pad     |
| Solder mask       | Green, both sides               |
| Silkscreen        | White, both sides               |

## Board-to-Board Connector

- Type: Hirose DF40 series (or equivalent), 0.4 mm pitch
- Pin count: 60 pins (signal + power + ground)
- Signals carried:
  - LVDS data pairs (4 differential pairs)
  - LVDS clock pair (1 differential pair)
  - SPI bus (SCK, MOSI, MISO, CS)
  - Power: 5V (for RF LDOs), GND
  - GPIO: reset, interrupt, status
- Mating height: 1.5 mm (allows RF board to sit parallel above main board)

## Design Notes

### FPGA Layout
- Place FPGA centrally on the board for symmetric routing to peripherals.
- Dedicated 1.0V and 0.9V power islands under FPGA with via stitching to inner ground plane.
- JTAG header (1.27 mm pitch, 2x7) placed on board edge for programming access.
- Configuration flash (W25Q128) placed within 15 mm of FPGA SPI pins.

### Power Integrity
- Each power rail has dedicated area on Layer 4 (power plane), with proper copper pours and via connections.
- Bulk capacitors (22 uF, 10 uF) near regulator outputs; 100 nF + 10 nF decoupling at each FPGA power pin.
- Ground planes on Layer 2 and Layer 5 provide solid return paths and shielding.

### Signal Integrity
- LVDS pairs: 100 ohm differential, length matched within 0.5 mm per pair, max trace length 30 mm.
- SPI to AWR2243: keep under 50 mm, series termination resistors (33 ohm) at source.
- IMU SPI: route on inner layer (L3) if needed, keep short (<20 mm).
- microSD traces: controlled impedance not critical at SPI speeds, but keep under 30 mm.

### Thermal
- FPGA exposed pad (if present) connected to ground plane with thermal via array.
- Thermal relief pads on power components for hand soldering rework capability.
- Thermal pad area under FPGA aligned with heat sink mounting in enclosure.

### Mechanical
- Mounting holes: 4x M3 at corners (3.2 mm drill), grounded.
- Board-to-board connector positioned to align with RF module mounting.
- FPC connector for display on top edge of board.
- USB-C connector on bottom edge for charging access through enclosure.
- microSD slot accessible from side of enclosure.
