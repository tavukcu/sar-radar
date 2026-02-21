# PCB Stackup Specifications

## Main Board - 6-Layer FR4 Stackup

Target stackup compatible with JLCPCB JLC2313 (standard 6-layer impedance controlled).

### Layer Stack

```
=================================================================
 Layer   |  Type    |  Material   | Thickness |  Function
=================================================================
 L1      | Signal   | Copper 1oz  | 35 um     | Top components + routing
         | Prepreg  | 2313        | 0.2104 mm |
 L2      | Plane    | Copper 1oz  | 35 um     | GND plane (continuous)
         | Core     | FR4         | 0.265 mm  |
 L3      | Signal   | Copper 1oz  | 35 um     | Inner signal routing
         | Prepreg  | 2313        | 0.2104 mm |
 L4      | Plane    | Copper 1oz  | 35 um     | Power plane (split)
         | Core     | FR4         | 0.265 mm  |
 L5      | Plane    | Copper 1oz  | 35 um     | GND plane (continuous)
         | Prepreg  | 2313        | 0.2104 mm |
 L6      | Signal   | Copper 1oz  | 35 um     | Bottom components + routing
=================================================================
 Total board thickness:  ~1.6 mm (+/- 10%)
=================================================================
```

### Layer Descriptions

**Layer 1 - Top Signal (Components)**
- Primary component placement side
- High-speed signal routing (LVDS, SPI)
- Microstrip traces referenced to L2 GND plane
- USB-C connector, FPGA, ESP32, IMU, display connector
- Power regulator placement with local ground pours

**Layer 2 - Ground Plane**
- Continuous, unbroken ground plane
- Primary reference plane for L1 microstrip impedance
- Return current path for all L1 high-speed signals
- Via stitching at board edges and around high-speed signal areas
- No splits or routing on this layer

**Layer 3 - Inner Signal**
- Secondary signal routing layer
- Stripline traces referenced to L2 (above) and L4 (below)
- IMU SPI bus, low-speed control signals, GPIO routing
- FPGA configuration flash SPI routing
- Avoid crossing L4 power plane splits with high-speed signals

**Layer 4 - Power Plane (Split)**
- Split power plane with the following regions:
  - 3.3V: largest area, covers FPGA I/O bank, ESP32, SD card, misc logic
  - 1.8V: medium area, covers FPGA auxiliary and LVDS bank
  - 1.0V: localized under FPGA for VCCINT
  - 0.9V: small area under FPGA for VCCBRAM
- Each region connected to its respective TPS62827 output via short, wide traces
- Plane splits arranged to avoid crossing high-speed signal paths on L3
- VBAT copper pour for battery distribution to regulators

**Layer 5 - Ground Plane**
- Second continuous ground plane
- Reference plane for L6 microstrip and L3/L4 shielding
- Provides cavity shielding between L4 power and L6 signals
- Via stitching matching L2 pattern

**Layer 6 - Bottom Signal (Components)**
- Secondary component placement
- Passive components (decoupling caps, resistors)
- microSD card slot
- Test points for debug access
- Board-to-board connector to RF module
- Battery connector

### Impedance Targets

| Trace Type        | Layers  | Target  | Width    | Spacing  | Reference     |
|-------------------|---------|---------|----------|----------|---------------|
| Single-ended      | L1, L6  | 50 ohm  | ~0.13 mm | N/A      | Microstrip to GND |
| Differential pair | L1, L6  | 100 ohm | ~0.10 mm | 0.15 mm  | Microstrip to GND |
| Single-ended      | L3      | 50 ohm  | ~0.09 mm | N/A      | Stripline L2/L4 |
| Differential pair | L3      | 100 ohm | ~0.08 mm | 0.12 mm  | Stripline L2/L4 |

Note: Exact trace widths must be verified with the fabricator's impedance calculator for the specific JLC2313 stackup and actual prepreg/core thicknesses.

### Design Rules (JLCPCB 6-Layer)

| Parameter              | Value          |
|------------------------|----------------|
| Minimum trace width    | 3.5 mil (0.089 mm) |
| Minimum trace spacing  | 3.5 mil (0.089 mm) |
| Minimum via drill      | 0.2 mm         |
| Minimum via pad        | 0.45 mm        |
| Minimum annular ring   | 0.125 mm       |
| Board thickness        | 1.6 mm         |
| Copper weight           | 1 oz (all layers) |
| Surface finish         | ENIG           |

---

## RF Module - 2-Layer Rogers RO4350B Stackup

Specialized high-frequency laminate for 77 GHz radar front-end operation.

### Layer Stack

```
=================================================================
 Layer   |  Type    |  Material     | Thickness |  Function
=================================================================
 L1      | Signal   | Copper 1oz    | 35 um     | RF traces + patch antennas
         | Core     | Rogers RO4350B| 0.254 mm  | (10 mil dielectric)
 L2      | Plane    | Copper 1oz    | 35 um     | Continuous ground plane
=================================================================
 Total board thickness:  ~0.324 mm (without soldermask)
=================================================================
```

### Material Properties (RO4350B at 77 GHz)

| Parameter                  | Value              |
|----------------------------|--------------------|
| Dielectric constant (Er)   | 3.66 +/- 0.05     |
| Loss tangent (tan_d)       | 0.0037 at 10 GHz  |
| Estimated tan_d at 77 GHz  | ~0.005             |
| Thermal conductivity       | 0.69 W/m/K        |
| CTE (x,y)                  | 11 ppm/C           |
| CTE (z)                    | 32 ppm/C           |
| Tg                         | >280C              |
| Copper peel strength       | >8 lb/in           |

### Layer Descriptions

**Layer 1 - RF Signal (Top)**
- AWR2243 BGA footprint and routing
- TX microstrip feed lines to 3 patch antenna elements
- RX microstrip feed lines from 4 patch antenna elements
- Patch antenna elements (rectangular, dimensions optimized at 77 GHz)
- Power supply decoupling (LDO output, bypass caps)
- SPI and LVDS traces (digital, routed away from RF)
- Board-to-board connector pads
- No solder mask over antenna patch area

**Layer 2 - Ground Plane (Bottom)**
- Continuous, unbroken copper ground plane
- AWR2243 thermal pad via connection (dense thermal via array)
- Board-to-board connector ground pads
- No routing on this layer

### RF Impedance

| Trace Type       | Target  | Width    | Notes                    |
|------------------|---------|----------|--------------------------|
| 50 ohm microstrip| 50 ohm  | ~0.55 mm | Calculated for 10 mil RO4350B |
| Patch feed       | 50 ohm  | ~0.55 mm | Quarter-wave transformer may be needed |

Note: At 77 GHz, dispersion and surface roughness effects are significant. Final trace widths should be verified with full-wave EM simulation (Ansys HFSS or similar). Copper surface roughness (Rz) should be specified to fabricator; use low-profile copper (LP) if available from JLCPCB Rogers service.

### Special Fabrication Notes for RF Board

- Request RO4350B from JLCPCB Rogers/high-frequency material service.
- Specify tight etch tolerance: +/- 0.5 mil (0.013 mm) for antenna elements.
- No solder mask on antenna side (Layer 1, antenna area only).
- Solder mask permitted on component area and Layer 2.
- Via fill and cap recommended for thermal vias under AWR2243.
- Panel layout: include coupon for impedance testing at 77 GHz if available.
