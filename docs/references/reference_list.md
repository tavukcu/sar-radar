# Reference List - 77 GHz Handheld SAR Radar Project

## 1. Component Datasheets and Technical References

### 1.1 Radar Transceiver

- **TI AWR2243 Datasheet**
  - Title: AWR2243 Single-Chip 76-GHz to 81-GHz FMCW Transceiver
  - Document: SWRS231
  - URL: https://www.ti.com/lit/ds/symlink/awr2243.pdf
  - Key content: Pin assignments, electrical specifications, timing diagrams, package thermal data

- **TI AWR2243 Technical Reference Manual**
  - Title: AWR2243 Technical Reference Manual
  - Document: SWRU553
  - URL: https://www.ti.com/lit/ug/swru553a/swru553a.pdf
  - Key content: Register maps, SPI protocol, chirp configuration, LVDS format, calibration procedures

- **TI DCA1000EVM User Guide**
  - Title: DCA1000EVM Data Capture Adapter Evaluation Module
  - Document: SPRUIJ4
  - URL: https://www.ti.com/lit/ug/spruij4a/spruij4a.pdf
  - Key content: Reference design for AWR2243 LVDS data capture, FPGA interface details, schematic reference

### 1.2 FPGA

- **Xilinx Artix-7 FPGA Datasheet**
  - Title: 7 Series FPGAs Data Sheet: Overview
  - Document: DS180
  - URL: https://docs.amd.com/v/u/en-US/ds180_7Series_Overview
  - Key content: XC7A35T specifications, BRAM count, I/O banks, LVDS support

- **Xilinx 7 Series FPGA SelectIO Resources User Guide**
  - Document: UG471
  - Key content: LVDS input configuration, ISERDES for high-speed data capture, clock management

- **Xilinx 7 Series DSP48E1 Slice User Guide**
  - Document: UG479
  - Key content: DSP slice architecture for FFT implementation, multiply-accumulate operations

### 1.3 IMU

- **TDK InvenSense ICM-42688-P Datasheet**
  - Title: ICM-42688-P High Precision 6-Axis MEMS MotionTracking Device
  - URL: https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf
  - Key content: Accelerometer/gyroscope specifications, SPI interface, bias instability, noise density

### 1.4 Wireless

- **Espressif ESP32-S3 Datasheet**
  - Title: ESP32-S3 Series Datasheet
  - URL: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
  - Key content: WiFi throughput, SPI master/slave, GPIO, power consumption modes

### 1.5 Power Management

- **TI TPS7A47 Ultra-Low-Noise LDO**
  - Document: SBVS081
  - Key content: 4.2 uVrms output noise, PSRR curves, thermal performance

- **TI TPS62160 Step-Down Converter**
  - Document: SLVSB41
  - Key content: 2 MHz switching, efficiency curves, layout guidelines

- **TI BQ25895 Battery Charger**
  - Document: SLUSBX3
  - Key content: USB-C charging, battery management, system power path

### 1.6 PCB Material

- **Rogers RO4350B Laminate Datasheet**
  - URL: https://www.rogerscorp.com/advanced-electronics-solutions/ro4000-series-laminates/ro4350b-laminates
  - Key content: Dielectric constant vs. frequency, loss tangent, thermal properties, standard thicknesses


## 2. SAR Signal Processing Textbooks

### 2.1 Primary References

- **Soumekh, M. (1999)**
  - Title: "Synthetic Aperture Radar Signal Processing with MATLAB Algorithms"
  - Publisher: Wiley-Interscience
  - ISBN: 978-0471297062
  - Key chapters: Ch. 2 (FMCW fundamentals), Ch. 4 (Stripmap SAR), Ch. 6 (Back-projection), Ch. 8 (Autofocus)
  - Relevance: MATLAB code examples directly applicable to our Python implementation

- **Carrara, W.G., Goodman, R.S., Majewski, R.M. (1995)**
  - Title: "Spotlight Synthetic Aperture Radar: Signal Processing Algorithms"
  - Publisher: Artech House
  - ISBN: 978-0890067284
  - Key chapters: Ch. 3 (Signal model), Ch. 5 (Polar format), Ch. 7 (Autofocus)
  - Relevance: Rigorous derivation of SAR signal models, spotlight mode theory

- **Cumming, I.G., Wong, F.H. (2005)**
  - Title: "Digital Processing of Synthetic Aperture Radar Data: Algorithms and Implementation"
  - Publisher: Artech House
  - ISBN: 978-1580530583
  - Key chapters: Ch. 6 (Range-Doppler), Ch. 9 (Back-projection), Ch. 11 (Autofocus), Ch. 14 (Motion compensation)
  - Relevance: Most comprehensive reference for SAR processing implementation details

### 2.2 Supplementary Textbooks

- **Richards, M.A. (2014)**
  - Title: "Fundamentals of Radar Signal Processing" (2nd Edition)
  - Publisher: McGraw-Hill
  - ISBN: 978-0071798327
  - Relevance: General radar signal processing, matched filtering, detection theory

- **Skolnik, M.I. (2008)**
  - Title: "Radar Handbook" (3rd Edition)
  - Publisher: McGraw-Hill
  - ISBN: 978-0071485470
  - Relevance: Comprehensive radar engineering reference, link budget, antenna design


## 3. Key Research Papers

### 3.1 Handheld SAR and FMCW SAR

- **Yanik, M.E., Torlak, M. (2020)**
  - Title: "Near-Field MIMO-SAR Millimeter-Wave Imaging with Sparsely Sampled Aperture Data"
  - Journal: IEEE Access, Vol. 7
  - DOI: 10.1109/ACCESS.2019.2950282
  - Relevance: MIMO-SAR imaging with TI mmWave sensors, directly applicable to our system

- **Gao, X., Xing, M., et al. (2018)**
  - Title: "A 77-GHz Portable FMCW Radar for Through-Wall Imaging"
  - Relevance: Portable 77 GHz SAR system design, similar concept to our handheld device

- **Alvarez, Y., Las-Heras, F. (2019)**
  - Title: "On the Use of FMCW Radar Sensors for SAR Imaging"
  - Relevance: FMCW-specific SAR signal model and processing considerations

### 3.2 Phase Gradient Autofocus

- **Wahl, D.E., Eichel, P.H., Ghiglia, D.C., Jakowatz, C.V. (1994)**
  - Title: "Phase Gradient Autofocus - A Robust Tool for High Resolution SAR Phase Correction"
  - Journal: IEEE Transactions on Aerospace and Electronic Systems, Vol. 30, No. 3
  - DOI: 10.1109/7.303752
  - Relevance: Original PGA paper, algorithm description and convergence analysis

- **Ash, J.N. (2012)**
  - Title: "An Autofocus Method for Backprojection Imagery in Synthetic Aperture Radar"
  - Journal: IEEE Geoscience and Remote Sensing Letters
  - Relevance: PGA adapted for back-projection SAR (our processing approach)

### 3.3 Motion Compensation and IMU Integration

- **Otten, M., et al. (2014)**
  - Title: "Motion Compensation for Hand-held SAR"
  - Relevance: Directly addresses the freehand scanning challenge with IMU-based correction

- **Fang, J., et al. (2015)**
  - Title: "A ZUPT-Aided INS/GPS Integration Algorithm for Ground Vehicle Navigation"
  - Relevance: ZUPT implementation details applicable to our handheld motion estimation

### 3.4 Back-Projection Algorithm

- **Ulander, L.M.H., Hellsten, H., Stenstrom, G. (2003)**
  - Title: "Synthetic-Aperture Radar Processing Using Fast Factorized Back-Projection"
  - Journal: IEEE Transactions on Aerospace and Electronic Systems, Vol. 39, No. 3
  - DOI: 10.1109/TAES.2003.1238734
  - Relevance: Fast BPA algorithm that reduces complexity from O(N^3) to O(N^2 log N)

- **McCorkle, J., Rofheart, M. (1996)**
  - Title: "An Order N^2 log N Backprojector Algorithm for Focusing Wide-Angle Wide-Bandwidth Arbitrary-Motion Synthetic Aperture Radar"
  - Relevance: Efficient BPA for GPU implementation

### 3.5 MIMO Radar and Virtual Arrays

- **Li, J., Stoica, P. (2007)**
  - Title: "MIMO Radar with Colocated Antennas"
  - Journal: IEEE Signal Processing Magazine
  - Relevance: Fundamental MIMO radar theory, virtual array concept

- **Zhuge, X., Yarovoy, A.G. (2012)**
  - Title: "Three-Dimensional Near-Field MIMO Array Imaging Using Range Migration Techniques"
  - Journal: IEEE Transactions on Image Processing
  - Relevance: MIMO array processing for near-field (short-range) SAR imaging


## 4. TI Application Notes and Resources

- **SWRA553**
  - Title: "Programming Chirp Parameters in TI Radar Devices"
  - Key content: Step-by-step chirp configuration, register programming sequence

- **SWRA564**
  - Title: "mmWave Radar Sensors - Antenna Design Guide"
  - Key content: Patch antenna design for TI radar sensors, array layout recommendations

- **SWRA581**
  - Title: "mmWave Sensor Raw Data Capture Using DCA1000 and mmWave Studio"
  - Key content: Raw ADC data format, LVDS capture setup, data parsing

- **TIDEP-01012**
  - Title: "Imaging Radar Using AWR2243 Cascade Reference Design"
  - Key content: Multi-chip cascaded radar design, relevant MIMO processing

- **TI mmWave SDK**
  - URL: https://www.ti.com/tool/MMWAVE-SDK
  - Key content: Driver libraries, signal processing chain reference, DPC (Data Path Chain) examples

- **TI Radar Toolbox (MATLAB)**
  - URL: https://www.ti.com/tool/MMWAVE-STUDIO
  - Key content: MATLAB-based radar signal processing, chirp design tools, visualization


## 5. FPGA Design Resources

- **Xilinx Application Note XAPP524**
  - Title: "Serial LVDS High-Speed ADC Interface"
  - Key content: LVDS deserialization techniques for Artix-7, clock recovery

- **Xilinx Application Note XAPP1285**
  - Title: "Radix-2 FFT Reference Design"
  - Key content: Pipelined FFT implementation for 7 Series FPGAs, resource utilization

- **Xilinx LogiCORE IP Fast Fourier Transform**
  - Document: PG109
  - Key content: Configurable FFT IP core, 512-point option, streaming architecture


## 6. PCB Design and Manufacturing

- **JLCPCB Rogers PCB Capabilities**
  - URL: https://jlcpcb.com/capabilities/pcb-capabilities
  - Key content: Supported Rogers materials, minimum features, layer stackup options, pricing

- **Rogers Corporation Design Calculators**
  - URL: https://www.rogerscorp.com/advanced-electronics-solutions/impedance-calculator
  - Key content: Microstrip impedance calculator for RO4350B, validated at mmWave frequencies

- **IPC-2221B**
  - Title: "Generic Standard on Printed Board Design"
  - Key content: General PCB design rules, clearances, via sizing

- **IPC-2226**
  - Title: "Sectional Design Standard for High Density Interconnect (HDI) Printed Boards"
  - Key content: HDI design rules relevant to via-in-pad and fine-pitch components


## 7. Safety Standards

- **IEEE C95.1-2019**
  - Title: "IEEE Standard for Safety Levels with Respect to Human Exposure to Electric, Magnetic, and Electromagnetic Fields"
  - Key content: Exposure limits at 77 GHz, power density thresholds for occupational and general public

- **FCC Part 15.255**
  - Title: "Operation within the band 76-81 GHz"
  - Key content: Unlicensed operation rules for 77 GHz radar, EIRP limits, restrictions

- **ETSI EN 302 264**
  - Title: "Short Range Devices; Transport and Traffic Telematics (TTT); Short Range Radar at 77 GHz"
  - Key content: European regulatory framework for 77 GHz radar devices


## 8. Software and Tools

- **Python Libraries Used in Simulation**
  - NumPy >= 1.21: Array operations, FFT
  - SciPy >= 1.7: Signal processing (windows, filters)
  - Matplotlib >= 3.5: Visualization and plotting
  - CuPy (optional): GPU-accelerated NumPy for CUDA back-projection

- **FPGA Development Tools**
  - Xilinx Vivado Design Suite (WebPACK edition, free for Artix-7)
  - Xilinx Vitis HLS (for C-to-RTL of FFT algorithms)

- **PCB Design Tools**
  - KiCad 8.x (open source, supports Rogers material stackups)
  - Ansys HFSS or CST Microwave Studio (for antenna simulation, if available)

- **Radar Simulation and Analysis**
  - MATLAB Radar Toolbox (commercial, for validation)
  - GNU Octave (free MATLAB alternative, compatible with most radar scripts)
  - OpenRADSimulator (open source radar simulation framework)
