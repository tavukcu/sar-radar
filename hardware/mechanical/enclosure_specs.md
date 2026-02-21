# Enclosure Specifications - Handheld SAR Radar

## Overview

The enclosure is designed for comfortable handheld operation of the 77 GHz SAR radar. It houses the main board, RF module, battery, display, and thermal management components in a compact, ergonomic form factor. The enclosure is 3D printed in PETG with a CNC-machined HDPE radome window for the antenna.

## External Dimensions

| Parameter          | Value              |
|--------------------|--------------------|
| Length             | 180 mm             |
| Width              | 90 mm              |
| Height (depth)     | 45 mm              |
| Weight (enclosure) | ~80 g (PETG)       |
| Weight (total)     | ~400-500 g (assembled with battery) |

## Materials

### Enclosure Body - PETG
- Manufacturing: FDM 3D printing
- Layer height: 0.2 mm
- Infill: 30% gyroid (balance of strength and weight)
- Wall thickness: 2.0 mm (minimum)
- Color: matte black (recommended for professional appearance)
- Post-processing: light sanding on grip surfaces, optional vapor smoothing

**Why PETG:**
- Higher impact resistance than PLA
- Better heat resistance (Tg ~80C) for sustained operation
- Good chemical resistance
- Easy to print with standard FDM printers
- Low cost for prototyping and iteration

### Radome - HDPE
- Manufacturing: CNC machined from HDPE sheet stock
- Dimensions: 55 x 45 x 2 mm (covers antenna array with margin)
- Thickness: 2.0 mm
- Surface finish: smooth both sides (Ra < 0.8 um)

**HDPE at 77 GHz:**
- Dielectric constant: Er ~ 2.3
- Loss tangent: tan_d ~ 0.0003
- Insertion loss through 2 mm: < 0.3 dB (very low)
- The radome is essentially transparent to 77 GHz radiation
- Thickness chosen to avoid resonant reflection effects (not lambda/2 in material)

**Mounting:** HDPE radome press-fit or glued (cyanoacrylate) into a recessed pocket on the front face of the enclosure, flush with exterior surface.

## Internal Layout

```
 TOP VIEW (front face up)
 +-------------------------------------------------+
 |  [Radome Window]                                |
 |  +-------------------+                          |
 |  |  RF Module        |                          |
 |  |  (50x40mm)        |  <- board-to-board conn  |
 |  +-------------------+                          |
 |  |  Heat Sink        |                          |
 |  |  (40x40x3mm Al)   |                          |
 |  +-------------------+                          |
 |  |  Main Board       |   [Display]              |
 |  |  (80x60mm)        |   [cutout]               |
 |  |                   |                          |
 |  +-------------------+                          |
 |  +-------------------+                          |
 |  |  2S Li-Po Battery |                          |
 |  |  (4000mAh)        |                          |
 |  +-------------------+                          |
 |                          [USB-C]   [Buttons x3] |
 +-------------------------------------------------+
                            (bottom edge)
```

### Component Stacking (cross-section, front to back)
1. Radome (HDPE, 2 mm) - flush with front face
2. Air gap (~1 mm)
3. RF module PCB (0.3 mm) - antenna side facing radome
4. Board-to-board connector gap (1.5 mm)
5. Heat sink plate (3 mm aluminum)
6. Thermal pad (0.5 mm)
7. Main board PCB (1.6 mm) - FPGA side facing heat sink
8. Battery compartment (~10 mm)
9. Enclosure back panel (2 mm PETG)

## Features

### Ergonomic Grip
- Contoured lower section (battery area) for comfortable one-handed operation
- Textured grip surface (printed ridges or rubber grip tape applied post-print)
- Center of gravity positioned near grip center for balanced feel
- Designed for right-hand or left-hand operation (symmetric grip)

### Button Cutouts
- 3 tactile button positions on the bottom edge or side
- Button functions: Power On/Off, Scan Start/Stop, Mode Select
- Cutout dimensions: 7 x 7 mm each, 2 mm recess for button caps
- Silicone button caps (aftermarket) for weatherproofing and tactile feel
- Button spacing: 15 mm center-to-center

### USB-C Port
- Cutout on bottom edge: 9.5 x 3.5 mm
- Aligned with USB-C connector on main board
- Recessed 1 mm for cable strain relief
- Accessible for charging and data transfer

### Ventilation Slots
- Location: both sides of enclosure, near heat sink area
- Slot dimensions: 1.5 x 15 mm, 6 slots per side
- Purpose: passive convective cooling for FPGA and AWR2243
- Slots angled downward (when held horizontally) to prevent dust/debris ingress

### Display Window
- Cutout on top face (or rear face): 30 x 30 mm for 1.3" TFT
- Covered with clear PETG or polycarbonate window (0.5 mm)
- Bonded with optically clear adhesive

### microSD Access
- Slot on right side of enclosure
- Spring-loaded door or rubber plug for dust protection
- Aligned with microSD socket on main board

## Mounting System

### Brass Heat-Set Inserts
- Type: M3 x 5 mm length x 5 mm OD
- Quantity: 8 total
  - 4 for enclosure top/bottom shell fastening
  - 4 for main board mounting (aligned with board mounting holes)
- Installation: soldering iron with tapered tip, pressed into PETG at ~240C
- Provides durable, reusable metal threads in plastic

### PCB Mounting
- Main board: 4x M3 screws into heat-set inserts, with 3 mm standoffs
- RF module: mounted to main board via board-to-board connector (self-supporting)
- Additional support post under RF module center to prevent flex

### Heat Sink Mounting
- Aluminum plate (40 x 40 x 3 mm) bonded to FPGA thermal pad with thermal adhesive
- Heat sink bridges between main board FPGA and RF module AWR2243
- Thermal interface material (silicone pad, 0.5 mm) on both sides
- Secured by compression from enclosure shell clamping

### Battery Mounting
- Battery sits in a pocket in the lower enclosure half
- Foam padding (1 mm) on sides to prevent rattling
- Battery wire routed through channel to main board connector
- Battery is not user-replaceable (requires opening enclosure)

## Enclosure Assembly

### Shell Design
- Two-piece clamshell (top shell + bottom shell)
- Split line along the long edge, midway through height
- Alignment features: 4 locating pins/sockets on mating surface
- Sealed with M3 screws (4 total) into heat-set inserts in bottom shell

### Assembly Order
1. Install heat-set inserts into bottom shell (8 total)
2. Place battery in pocket, route wire
3. Mount main board on standoffs with M3 screws (4x)
4. Attach board-to-board connector and seat RF module
5. Place thermal pads and heat sink
6. Connect display FPC cable
7. Bond radome into front face pocket
8. Bond display window
9. Close top shell, fasten with M3 screws (4x)

## Environmental

| Parameter            | Target                |
|----------------------|-----------------------|
| Operating temp       | 0C to 45C             |
| Storage temp         | -20C to 60C           |
| Humidity             | Up to 80% RH non-condensing |
| IP rating            | IP20 (indoor/dry use) |
| Drop resistance      | 1 m onto concrete (with battery) |

## Future Improvements

- Injection molded enclosure (ABS/PC blend) for production volumes
- IP54 rating with gaskets and sealed connectors
- Integrated battery door for field replacement
- Lanyard attachment point
- Tripod mount (1/4"-20 thread insert) for stationary scanning
