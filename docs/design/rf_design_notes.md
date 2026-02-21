# RF Design Notes - 77 GHz Handheld SAR Radar

## 1. 77 GHz PCB Design Challenges

### 1.1 Wavelength Considerations

| Medium | Wavelength at 77 GHz | Notes |
|---|---|---|
| Free space | 3.896 mm | lambda_0 = c / f |
| RO4350B (er=3.66) | 2.036 mm | lambda_g = lambda_0 / sqrt(er_eff) |
| RO4350B microstrip (er_eff~2.9) | 2.288 mm | Accounts for fringing fields |
| FR4 (er~4.4) | 1.857 mm | NOT suitable at 77 GHz (loss > 30 dB/cm) |

At 77 GHz, a quarter wavelength on the PCB is approximately 0.5 mm. This means:
- Every millimeter of trace matters for impedance and phase
- Component placement tolerances become critical
- Via transitions must be carefully designed to minimize discontinuities
- Standard FR4 PCB material is completely unsuitable (loss tangent ~0.02 at 77 GHz)

### 1.2 Substrate Selection: Rogers RO4350B

**Why RO4350B:**
- Low loss tangent: 0.0037 at 10 GHz (approximately 0.005-0.008 at 77 GHz)
- Stable dielectric constant: er = 3.66 +/- 0.05
- Compatible with standard FR4 processing (no special equipment needed)
- Available from JLCPCB as a Rogers material option
- Temperature stable: TCDk = +50 ppm/C

**Alternative considered: Rogers RT5880:**
- Even lower loss (tan_d = 0.0009) but:
- Very soft material (difficult to handle at thin dimensions)
- Worse dimensional stability
- Higher cost, limited JLCPCB availability
- er = 2.2 (wider traces needed)

**Thickness selection: 0.254 mm (10 mil)**
- Thin enough for reasonable trace widths at 77 GHz
- Standard Rogers core thickness
- Available from JLCPCB

### 1.3 Microstrip Impedance at 77 GHz

For 50 ohm microstrip on 0.254 mm RO4350B:

Using the Hammerstad-Jensen formula:

```
For W/h >= 1:
  Z0 = (120*pi / sqrt(er_eff)) * 1 / (W/h + 1.393 + 0.667*ln(W/h + 1.444))

Solving for W at Z0 = 50 ohm, er = 3.66, h = 0.254 mm:
  W_50ohm ~ 0.55 mm (including frequency dispersion effects)

At 77 GHz with dispersion correction:
  W_50ohm ~ 0.50 mm (narrower due to increased er_eff at high frequency)
```

**Practical value: W = 0.15 mm (for 50 ohm at 77 GHz on 0.254 mm RO4350B)**

Note: The significant narrowing from low-frequency estimates is due to the strong frequency dispersion of the effective dielectric constant at millimeter-wave frequencies. Full-wave simulation (HFSS or CST) is essential for accurate impedance determination.

### 1.4 Manufacturing Constraints (JLCPCB Rogers)

| Parameter | JLCPCB Capability | Our Design | Margin |
|---|---|---|---|
| Minimum trace width | 0.1 mm (4 mil) | 0.15 mm | OK |
| Minimum trace spacing | 0.1 mm (4 mil) | 0.15 mm | OK |
| Via drill size | 0.2 mm minimum | 0.25 mm | OK |
| Via pad size | 0.45 mm minimum | 0.5 mm | OK |
| Registration accuracy | +/- 0.075 mm | - | Adequate |
| Copper thickness | 1 oz (35 um) or 0.5 oz (18 um) | 0.5 oz for RF | OK |
| Layer count | Up to 6 layers with Rogers | 4 layers (hybrid) | OK |


## 2. Patch Antenna Design

### 2.1 Rectangular Microstrip Patch Dimensions

Operating frequency: 77 GHz
Substrate: RO4350B, er = 3.66, h = 0.254 mm

**Patch Width (W):**

```
W = c / (2 * f0) * sqrt(2 / (er + 1))
  = 3e8 / (2 * 77e9) * sqrt(2 / (3.66 + 1))
  = 1.948 mm * 0.656
  = 1.278 mm

Practical (after simulation optimization): W ~ 1.45 mm
```

**Effective Dielectric Constant:**

```
er_eff = (er + 1)/2 + (er - 1)/2 * (1 + 12*h/W)^(-0.5)
       = (3.66 + 1)/2 + (3.66 - 1)/2 * (1 + 12*0.254/1.45)^(-0.5)
       = 2.33 + 1.33 * 0.647
       = 3.191
```

**Fringing Extension:**

```
dL = 0.412 * h * (er_eff + 0.3)*(W/h + 0.264) / ((er_eff - 0.258)*(W/h + 0.8))
   = 0.412 * 0.254 * (3.191 + 0.3)*(5.709 + 0.264) / ((3.191 - 0.258)*(5.709 + 0.8))
   = 0.1047 * 3.491 * 5.973 / (2.933 * 6.509)
   = 0.1047 * 20.85 / 19.09
   = 0.1144 mm
```

**Patch Length (L):**

```
L = c / (2 * f0 * sqrt(er_eff)) - 2 * dL
  = 3e8 / (2 * 77e9 * sqrt(3.191)) - 2 * 0.1144
  = 1.091 - 0.229
  = 0.862 mm

Practical (after simulation): L ~ 0.93 mm
```

**Summary of patch dimensions:**

```
+---------------------------+
|                           |
|     W = 1.45 mm           |  <-- Patch width (radiating edges)
|                           |
|          L = 0.93 mm      |  <-- Patch length (resonant dimension)
|                           |
|          ^                |
|          |                |
|     Feed point            |
+---------------------------+
          |
     Microstrip feed
     (W = 0.15 mm, 50 ohm)
```

### 2.2 Feed Techniques

**Option 1: Microstrip Edge Feed (Inset Feed)**

```
     +---------------------------+
     |                           |
     |       Patch               |
     |                           |
     |     +---+                 |
     |     |   | y0 (inset)     |
     +-----+   +---------+------+
               |
          50-ohm line
```

- Inset distance y0 controls impedance matching
- At the edge, impedance is ~200-300 ohm
- Inset moves toward center to reach 50 ohm
- Simple to fabricate, single layer
- `y0 ~ L/pi * arccos(sqrt(50/R_edge))`

**Option 2: Aperture-Coupled Feed**

```
     +---------------------------+  Patch (top layer)
     |         Patch             |
     +---------------------------+
     =====  Ground plane  =======
             [   slot   ]           Coupling slot in ground
     ___________________________
     |    Microstrip feed       |   Feed line (bottom layer)
     |__________________________|
```

- Better bandwidth than edge feed
- Cleaner radiation pattern (feed radiation reduced)
- Requires multilayer PCB (adds cost)
- Preferred for 77 GHz designs due to reduced parasitic radiation

**Chosen approach: Microstrip inset feed** for simplicity and JLCPCB compatibility.

### 2.3 Array Element Spacing

```
Element spacing: d = lambda_0 / 2 = 3.896 / 2 = 1.948 mm ~ 1.95 mm
```

This provides:
- Grating-lobe-free scanning up to +/- 90 degrees
- Manageable physical spacing for patch elements (1.45 mm wide patches with 0.5 mm gaps)

### 2.4 Array Layouts

**TX Array (3 elements, 2d spacing for MIMO):**

```
     TX0          TX1          TX2
  +-------+    +-------+    +-------+
  | Patch |    | Patch |    | Patch |
  +---+---+    +---+---+    +---+---+
      |            |            |
  |<--3.90mm-->|<--3.90mm-->|
  |<---------- 7.80 mm -------->|
```

**RX Array (4 elements, d spacing):**

```
  RX0      RX1      RX2      RX3
+-----+  +-----+  +-----+  +-----+
|Patch|  |Patch|  |Patch|  |Patch|
+--+--+  +--+--+  +--+--+  +--+--+
   |        |        |        |
|<1.95>|<1.95>|<1.95>|
|<------- 5.85 mm -------->|
```

### 2.5 Estimated Antenna Performance

| Parameter | Single Patch | TX Array (3el) | RX Array (4el) | MIMO Virtual (12el) |
|---|---|---|---|---|
| Gain | 5-6 dBi | 10-11 dBi | 11-12 dBi | 16-17 dBi |
| E-plane BW | ~80 deg | ~80 deg | ~80 deg | ~80 deg |
| H-plane BW | ~90 deg | ~25 deg | ~20 deg | ~8 deg |
| Bandwidth | ~5% (~3.85 GHz) | ~4% | ~4% | ~4% |
| Efficiency | ~85% | ~80% | ~80% | ~75% |
| Sidelobe level | N/A | -13 dB | -13 dB | -13 dB (uniform) |


## 3. AWR2243 RF Interface

### 3.1 Overview

The TI AWR2243 is a 76-81 GHz automotive radar transceiver with:
- 3 transmit channels
- 4 receive channels
- Internal PLL for chirp generation
- Built-in ADC (12-bit, 10 MSPS per channel)
- LVDS digital output

### 3.2 TX Output Specifications

| Parameter | Value | Notes |
|---|---|---|
| Frequency range | 76 - 81 GHz | Covers automotive radar band |
| TX output power | 12 dBm (per channel) | At antenna port |
| Number of TX | 3 | Independent control, TDM-MIMO |
| Output impedance | 50 ohm (differential) | Requires balun or diff antenna |
| Chirp bandwidth | Up to 5 GHz | Within 76-81 GHz |
| Chirp slope | Up to 250 MHz/us | Our design: 83.3 MHz/us |
| Phase noise | -95 dBc/Hz @ 1 MHz offset | Good for SAR coherence |

### 3.3 RX Input Specifications

| Parameter | Value | Notes |
|---|---|---|
| Noise figure | 12 dB (typical) | Including LNA |
| Number of RX | 4 | Simultaneous reception |
| Input impedance | 50 ohm (differential) | |
| RX gain | 42 dB (total chain) | LNA + Mixer + IF amp |
| IF bandwidth | 20 MHz max | Our design: 10 MHz |
| ADC resolution | 12 bits | Complex (I/Q) output |
| ADC sample rate | Up to 12.5 MSPS | Our design: 10 MSPS |

### 3.4 LVDS Digital Output

| Parameter | Value | Notes |
|---|---|---|
| Interface | 4 LVDS pairs (data) + 1 clock | Directly to FPGA |
| Data rate | Up to 600 Mbps per pair | DDR mode |
| Format | 12-bit packed or 16-bit | We use 16-bit for simplicity |
| Frame structure | Chirp data + header | Configurable via SPI |

### 3.5 PLL / Chirp Configuration

The AWR2243's internal PLL generates the FMCW chirp with the following configuration for our system:

```
Chirp Parameters:
  Start frequency: 76.0 GHz
  Stop frequency:  81.0 GHz
  Bandwidth:       5.0 GHz
  Chirp duration:  60 us
  Idle time:       20 us (between chirps)
  Ramp slope:      83.33 MHz/us
  ADC start:       5 us after chirp start (settling time)
  ADC samples:     512 per chirp
  ADC rate:        10 MSPS

Profile register values (approximate):
  CHIRP_START_FREQ = 0x5C28 (76.0 GHz)
  CHIRP_FREQ_SLOPE = 0x0D55 (83.33 MHz/us)
  CHIRP_IDLE_TIME  = 0x0014 (20 us)
  CHIRP_ADC_START  = 0x0005 (5 us)
  ADC_NUM_SAMPLES  = 0x0200 (512)
```

### 3.6 AWR2243 PCB Layout Guidelines

1. **Differential TX/RX traces**: Must be routed as 100 ohm differential pairs
2. **Trace length matching**: TX and RX traces to antenna must be matched within 0.1 mm
3. **Ground vias**: Stitching vias every 0.5 mm along RF traces (< lambda/4)
4. **Keep-out zones**: 2 mm clearance around antenna patches (no copper, no vias)
5. **Crystal**: 40 MHz crystal must be placed within 5 mm of AWR2243, with solid ground underneath
6. **Thermal**: Exposed pad must have multiple thermal vias to inner ground plane


## 4. Power Supply Filtering for RF

### 4.1 Power Supply Architecture for AWR2243

The AWR2243 requires multiple supply voltages with strict noise requirements:

| Supply | Voltage | Current | Noise Req. | Regulator | Notes |
|---|---|---|---|---|---|
| VDD_1P2_RF | 1.2 V | 400 mA | < 10 uVrms | ADP1720-1.2 | RF PLL, synthesizer |
| VDD_1P2_DIG | 1.2 V | 300 mA | < 50 uVrms | ADP1720-1.2 | Digital core |
| VDD_1P8_RF | 1.8 V | 200 mA | < 20 uVrms | TPS7A47-1.8 | LNA, mixer bias |
| VDD_1P8_IO | 1.8 V | 100 mA | < 100 uVrms | TPS7A47-1.8 | LVDS I/O |
| VDD_3P3_IO | 3.3 V | 50 mA | < 200 uVrms | Standard LDO | SPI, GPIO |

### 4.2 Decoupling Strategy

For each power pin of the AWR2243:

```
Power rail ----[Ferrite Bead]----+----[100nF]----GND
                                 |
                                 +----[10nF]-----GND
                                 |
                                 +----[1nF]------GND
                                 |
                                 +----[100pF]----GND (optional, for mmWave)
                                 |
                              AWR2243 pin
```

**Ferrite bead selection:**
- Murata BLM15PX601SN1D (600 ohm at 100 MHz)
- Low DC resistance (< 0.5 ohm to minimize voltage drop)
- Placed between regulator output and decoupling capacitor cluster

**Capacitor selection at 77 GHz:**
- 100 nF: 0402 package, X5R, low ESL (provides bulk decoupling)
- 10 nF: 0201 package, C0G/NP0 (resonant around 200-500 MHz)
- 1 nF: 0201 package, C0G/NP0 (resonant around 1-3 GHz)
- 100 pF: 0201 package, C0G/NP0 (resonant at higher frequencies)

**Critical**: At 77 GHz, the PCB trace inductance between capacitor and pin must be minimized. Place all decoupling capacitors within 0.5 mm of the AWR2243 power pins. Use via-in-pad for ground connections.

### 4.3 TPS7A47 Ultra-Low Noise LDO

Selected for the critical 1.8V analog supply:

| Parameter | Value |
|---|---|
| Output voltage | 1.8 V (fixed) |
| Max output current | 500 mA |
| Dropout voltage | 310 mV @ 500 mA |
| Output noise | 4.2 uVrms (10 Hz - 100 kHz) |
| PSRR | 72 dB @ 1 kHz, 54 dB @ 100 kHz |
| Package | SOT-23-5 |

**Why this LDO**: The ultra-low noise (4.2 uVrms) ensures the AWR2243's internal PLL and LNA are not degraded by supply noise. Standard LDOs have 20-50 uVrms noise, which could add phase noise to the chirp.

### 4.4 Ground Plane Design

```
Layer Stack (4-layer hybrid):

Layer 1 (Top):      Signal + Patch antennas (RO4350B, 0.254 mm)
Layer 2 (Inner 1):  Continuous ground plane (critical for RF)
Layer 3 (Inner 2):  Power planes (split for analog/digital)
Layer 4 (Bottom):   Signal + Components (FR4, standard)

Bonding: RO4350B core for layers 1-2, FR4 prepreg for 2-3, FR4 core for 3-4
```

**Ground plane rules:**
1. Layer 2 must be an unbroken ground plane under all RF traces and antennas
2. No signal routing on Layer 2 in the RF section
3. Analog and digital ground planes are split on Layer 3, connected at a single point near the AWR2243
4. Ground via fence around the perimeter of the RF section (0.5 mm pitch)
5. Via stitching under microstrip lines (0.5 mm pitch, both sides of trace)

### 4.5 EMI Considerations

- 77 GHz radiation is highly directional (patch antenna) -- minimal stray radiation
- LVDS pairs must be routed as differential with ground shielding vias
- SPI clock (25 MHz) harmonics can potentially interfere -- route away from RX antenna
- Battery switching regulators (TPS62160, 2 MHz switching) must be physically separated from RF section (> 15 mm)
- Shielding can for the AWR2243 section is recommended for production units


## 5. PCB Fabrication Notes for JLCPCB

### 5.1 Material Specification

```
Layer 1-2: Rogers RO4350B, 0.254 mm (10 mil), 0.5 oz copper
Layer 2-3: FR4 prepreg, 0.2 mm
Layer 3-4: FR4 core, 0.8 mm, 1.0 oz copper

Total board thickness: ~1.5 mm
```

### 5.2 Special Instructions for JLCPCB Order

1. Specify Rogers RO4350B for Layer 1-2 core (not FR4)
2. Request 0.5 oz copper on Layers 1-2 (smoother surface for mmWave)
3. Immersion gold (ENIG) surface finish required (not HASL -- too rough at 77 GHz)
4. Impedance-controlled manufacturing with 50 ohm microstrip specification
5. Minimum feature size: 0.1 mm trace/space on Rogers layers
6. Via-in-pad with cap plating for RF section

### 5.3 Estimated Loss Budget (PCB)

| Path | Length | Loss | Notes |
|---|---|---|---|
| AWR2243 TX to antenna | 5 mm | ~1.0 dB | Microstrip on RO4350B |
| Antenna to free space | N/A | ~0.5 dB | Mismatch + dielectric loss |
| RX antenna to AWR2243 | 5 mm | ~1.0 dB | Microstrip on RO4350B |
| **Total one-way** | | **~1.5 dB** | |
| **Total round-trip** | | **~3.0 dB** | Included in link budget |

Note: Microstrip loss on RO4350B at 77 GHz is approximately 0.15-0.20 dB/mm, dominated by conductor loss (skin effect in copper at 77 GHz, skin depth ~ 0.24 um).
