# Manufacturing & Assembly Notes

## 1. Main Board (FR4, 6-Layer)

### PCB Fabrication

| Parameter              | Specification              |
|------------------------|----------------------------|
| Fabricator             | JLCPCB                    |
| Material               | FR4 (Tg >= 170C)          |
| Layer count            | 6                          |
| Stackup                | JLC2313 (impedance controlled) |
| Board dimensions       | 80 x 60 mm                |
| Copper weight           | 1 oz (all layers)         |
| Surface finish         | ENIG (Immersion Gold)      |
| Solder mask            | Green, both sides          |
| Silkscreen             | White, both sides          |
| Minimum trace/space    | 3.5 / 3.5 mil             |
| Minimum via drill      | 0.2 mm                    |
| Board thickness        | 1.6 mm                    |
| Impedance control      | Yes (50 ohm microstrip, 100 ohm diff) |
| Panelization           | V-score or tab-route       |

### SMT Assembly (Main Board)

- **Service:** JLCPCB PCBA (standard parts from LCSC inventory)
- **Assembly side:** Top side primary, bottom side secondary (two-pass reflow)
- **Solder paste:** SAC305 (lead-free), Type 4 for fine-pitch
- **Reflow profile:** Lead-free standard, peak 245-250C

**JLCPCB PCBA Compatible Parts:**
- TPS62827DMQR (buck converters)
- TPS7A4700RGWR (LDO)
- ICM-42688-P (IMU)
- ESP32-C3-MINI-1 (WiFi module)
- W25Q128JVSIQ (SPI flash)
- All passive components (0603, 0402, 0201)
- Connectors (USB-C, microSD, FPC)
- LEDs, ESD protection, ferrite beads

**Parts Requiring Manual/Consigned Assembly:**
- XC7A100T-1CPG236C (FPGA) - verify LCSC availability; if not stocked, consign to JLCPCB or hand-solder with stencil + reflow oven
- BQ25792RWWR (charger IC) - check LCSC stock
- TPS25750DRGR (USB-C PD controller) - likely consigned from DigiKey

### Test Points (Main Board)

Place accessible test points (1.0 mm pads, labeled on silkscreen) on the following:

| Test Point | Signal        | Expected Value   |
|------------|---------------|------------------|
| TP1        | VBAT          | 6.0 - 8.4V      |
| TP2        | 3.3V rail     | 3.30V +/- 3%    |
| TP3        | 1.8V rail     | 1.80V +/- 3%    |
| TP4        | 1.0V rail     | 1.00V +/- 3%    |
| TP5        | 0.9V rail     | 0.90V +/- 3%    |
| TP6        | 5V USB input  | 5.0V / PD negotiated |
| TP7        | SPI_SCK (IMU) | Digital clock    |
| TP8        | SPI_SCK (AWR) | Digital clock    |
| TP9        | LVDS_CLK+     | Differential     |
| TP10       | LVDS_CLK-     | Differential     |
| TP11       | FPGA_DONE     | High when configured |
| TP12       | GND           | 0V reference     |

---

## 2. RF Board (Rogers RO4350B, 2-Layer)

### PCB Fabrication

| Parameter              | Specification              |
|------------------------|----------------------------|
| Fabricator             | JLCPCB (Rogers service)   |
| Material               | Rogers RO4350B             |
| Layer count            | 2                          |
| Board dimensions       | 50 x 40 mm                |
| Core thickness         | 0.254 mm (10 mil)         |
| Copper weight           | 1 oz (35 um)              |
| Surface finish         | ENIG                       |
| Solder mask            | Green, bottom side only    |
| Solder mask exclusion  | No mask on antenna patches (top side antenna area) |
| Silkscreen             | Bottom side only           |
| Etch tolerance         | +/- 0.5 mil (critical for 77 GHz) |
| Impedance control      | Yes (50 ohm microstrip)    |

### AWR2243 BGA Assembly

**CRITICAL: The AWR2243BRHHR is a BGA-161 package and requires reflow oven assembly. Hand soldering is NOT possible for BGA.**

**Reflow Assembly Procedure:**
1. Apply solder paste using a stainless steel stencil (0.12 mm / 5 mil thickness).
2. Stencil aperture design: 1:1 ratio for BGA pads, round openings.
3. Place AWR2243 using pick-and-place or manual BGA alignment tool with microscope.
4. Reflow with controlled thermal profile:

| Phase               | Temperature    | Duration    |
|----------------------|----------------|-------------|
| Preheat ramp         | 25C to 150C   | 60-90 sec   |
| Soak                 | 150C to 200C  | 60-90 sec   |
| Reflow (TAL)         | >217C         | 60-90 sec   |
| Peak temperature     | 245-250C      | 10-20 sec   |
| Cooling ramp         | 250C to 25C   | <6C/sec     |

**Important Notes:**
- Rogers RO4350B is compatible with lead-free reflow temperatures (Tg > 280C).
- Use nitrogen atmosphere if available to prevent copper oxidation on exposed antenna pads.
- After reflow, perform X-ray inspection of BGA solder joints if possible.
- If X-ray not available, check continuity on all accessible pins and verify SPI communication.

**Other Components on RF Board:**
- TPS7A4700 LDOs: standard reflow, no special handling
- Passive components (0402, 0603): standard reflow
- Board-to-board connector: standard reflow
- All non-BGA components can be reflowed in the same pass as AWR2243

### Test Points (RF Board)

| Test Point | Signal            | Expected Value     |
|------------|-------------------|--------------------|
| TP_RF1     | 1.8V analog       | 1.80V +/- 1%      |
| TP_RF2     | 1.0V core         | 1.00V +/- 1%      |
| TP_RF3     | SPI_CLK           | Digital            |
| TP_RF4     | LVDS_D0+          | Differential       |
| TP_RF5     | LVDS_D0-          | Differential       |
| TP_RF6     | NRESET            | High (3.3V)        |
| TP_RF7     | GND               | 0V reference       |

---

## 3. Stencil Requirements

### Main Board Stencil
- Material: stainless steel, laser-cut
- Thickness: 0.12 mm (5 mil) standard
- Electropolished aperture walls recommended for fine-pitch QFN/BGA
- Order with JLCPCB PCBA service (included)

### RF Board Stencil
- Material: stainless steel, laser-cut
- Thickness: 0.12 mm (5 mil) for BGA
- Custom order from JLCPCB or third-party stencil supplier
- Frame-mounted preferred for manual printing
- Aperture modifications: reduce BGA pad apertures to 90% if bridging occurs

---

## 4. Assembly Order & Bring-Up Sequence

### Step 1: Main Board Power Validation
1. Populate only power section (TPS62827 x4, TPS7A47, input caps, inductors, output caps).
2. Apply 7.4V to battery input pads.
3. Measure all power rails at test points (TP1-TP5).
4. Verify power sequencing with oscilloscope (3.3V -> 1.8V -> 1.0V -> 0.9V).
5. Measure ripple on each rail (target: <20 mVpp for digital, <5 mVpp for analog).

### Step 2: FPGA Bring-Up
1. Populate FPGA, configuration flash, 100 MHz crystal, decoupling caps.
2. Connect JTAG programmer.
3. Verify FPGA is detected (Vivado Hardware Manager).
4. Program test bitstream (LED blink or UART loopback).
5. Verify FPGA_DONE signal goes high.

### Step 3: Peripheral Integration
1. Populate IMU, ESP32, display connector, microSD slot.
2. Test SPI communication with IMU (read WHO_AM_I register, expected: 0x47).
3. Test ESP32 UART communication.
4. Test display initialization (ST7789 color fill test).
5. Test microSD card detection and SPI read/write.

### Step 4: USB-C & Charging
1. Populate BQ25792, TPS25750, USB-C connector, ESD protection.
2. Connect USB-C PD source.
3. Verify PD negotiation (9V or 12V on VBUS).
4. Connect battery; verify charging LED and charge current.

### Step 5: RF Module Assembly
1. Apply stencil, paste, and reflow AWR2243 + all RF board components.
2. Visual and/or X-ray inspection of BGA joints.
3. Verify RF board power rails (TP_RF1, TP_RF2).
4. Connect RF module to main board via board-to-board connector.
5. Test SPI communication with AWR2243 (read device ID register).

### Step 6: System Integration
1. Verify LVDS data capture from AWR2243 through FPGA.
2. Run chirp sequence, capture raw ADC data to microSD.
3. Verify IMU data acquisition concurrent with radar operation.
4. Test WiFi data streaming via ESP32.
5. Full SAR scan test with motion (verify IMU + radar synchronization).

---

## 5. Quality Checklist

- [ ] All power rails within specification
- [ ] No solder bridges on fine-pitch components (visual inspection)
- [ ] FPGA configuration successful
- [ ] SPI communication verified (IMU, AWR2243, flash)
- [ ] LVDS data integrity (bit error rate test)
- [ ] USB-C PD negotiation functional
- [ ] Battery charge/discharge cycle test
- [ ] Thermal validation under full load (FPGA + AWR2243 < 85C junction)
- [ ] RF board antenna pattern (if anechoic chamber available)
- [ ] Current consumption measurement (target: <3W average, <5W peak)
