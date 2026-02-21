# RF Module - 77 GHz SAR Radar Front-End

## Overview

The RF module is the radar front-end board built around the Texas Instruments AWR2243 77 GHz FMCW radar MMIC. It integrates the transmitter chain, receiver chain, and patch antenna array on a single Rogers RO4350B substrate for optimal RF performance at millimeter-wave frequencies.

## Key Components

### AWR2243BRHHR Radar MMIC
- 76-81 GHz FMCW radar transceiver
- 3 transmit (TX) channels, 4 receive (RX) channels
- Integrated PLL, ADC (12-bit, 25 Msps per channel), and chirp sequencer
- BGA package with exposed thermal pad
- SPI configuration interface
- LVDS digital data output (up to 600 Mbps)

### Patch Antenna Array (3TX / 4RX)
- Operating frequency: 77 GHz center
- Element type: rectangular microstrip patch
- Patch spacing: lambda/2 at 77 GHz (~1.95 mm)
- TX array: 3 elements, linear arrangement for MIMO virtual aperture
- RX array: 4 elements, uniform linear array (ULA)
- Feed: microstrip edge-coupled feed, impedance matched to 50 ohm
- Estimated element gain: ~6 dBi per patch
- Total array gain (RX): ~12 dBi

### Power Regulation
- **TPS7A4700RGWR** ultra-low noise LDO regulator
  - Input: 5V from main board connector
  - Output: 1.8V analog supply for AWR2243
  - PSRR: >60 dB at 1 MHz
  - Noise: 4.17 uVrms (10 Hz to 100 kHz)
- Separate LDO instance for 1.0V core supply
- Ferrite bead isolation on all power inputs
- Decoupling: 100nF MLCC at each power pin, 10uF bulk per rail

### LVDS Data Output Connector
- High-speed LVDS pairs for ADC data streaming to FPGA
- Board-to-board connector (mating with main board)
- 100 ohm differential impedance, length matched within 0.1 mm
- Connector also carries SPI control lines, clock, and power

## Board Specifications

| Parameter         | Value                          |
|-------------------|--------------------------------|
| Substrate         | Rogers RO4350B                 |
| Layer count       | 2                              |
| Board dimensions  | 50 x 40 mm                     |
| Dielectric const. | Er = 3.66                      |
| Loss tangent      | tan_d = 0.0037 at 10 GHz       |
| Core thickness    | 0.254 mm (10 mil)              |
| Copper weight     | 1 oz (35 um)                   |
| Surface finish    | ENIG                           |

## Design Notes

### Controlled Impedance
- All RF traces are 50 ohm microstrip on Layer 1 over continuous ground plane on Layer 2.
- Microstrip width calculated for RO4350B at 10 mil core: approximately 0.55 mm for 50 ohm.
- Use EM solver (HFSS or ADS Momentum) to verify impedance at 77 GHz including dispersion effects.

### Antenna Spacing
- Free-space wavelength at 77 GHz: lambda_0 = 3.896 mm
- Guided wavelength in substrate: lambda_g = lambda_0 / sqrt(Er_eff) ~ 2.4 mm
- Element spacing (lambda/2 free-space): ~1.95 mm
- Tight pitch requires precision etching; specify minimum trace/space of 0.1 mm (4 mil) to fabricator.

### Layout Guidelines
- Keep all digital routing (SPI, LVDS) on the opposite side or edge of the board from the antenna array.
- Place AWR2243 as close to patch feeds as possible to minimize feed line loss.
- Use via stitching around antenna ground plane at lambda/20 spacing (~0.2 mm pitch) to suppress surface wave modes.
- No solder mask over antenna patches (bare copper or ENIG).
- Include calibration test structures: thru-line, open stub, and coupled-line coupler for production verification.

### Thermal Management
- AWR2243 thermal pad must be connected to ground plane with a grid of thermal vias (0.3 mm diameter, 0.6 mm pitch).
- Expected power dissipation: ~2.5 W (all TX active).
- Thermal interface material (TIM) between bottom ground plane and aluminum heat spreader in enclosure.

### Grounding
- Continuous ground plane on Layer 2, no splits or cuts.
- All component ground pads connected with multiple vias to ground plane.
- Board-to-board connector ground pins distributed evenly for low-impedance return path.

## Schematic Blocks

1. **AWR2243 Core** - MMIC with all bypass, crystal (40 MHz), and configuration resistors
2. **TX Chain** - 3 TX outputs to patch antenna feeds via microstrip
3. **RX Chain** - 4 RX inputs from patch antenna elements via microstrip
4. **Power** - TPS7A47 LDOs (1.8V analog, 1.0V core), ferrite beads, bulk/bypass caps
5. **Digital Interface** - SPI (config), LVDS (data), GPIO, clock
6. **Connector** - Board-to-board mating connector with power, data, and control signals
