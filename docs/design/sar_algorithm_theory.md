# SAR Algorithm Theory - 77 GHz Handheld Radar

## 1. FMCW Radar Basics

### 1.1 Chirp Signal

An FMCW (Frequency Modulated Continuous Wave) radar transmits a linear frequency sweep (chirp):

```
f_TX(t) = fc + (BW / T_chirp) * t ,   0 <= t <= T_chirp
```

Where:
- `fc` = 78.5 GHz (center frequency, within 76-81 GHz automotive band)
- `BW` = 5 GHz (chirp bandwidth)
- `T_chirp` = 60 us (chirp duration)
- Chirp slope: `S = BW / T_chirp = 5e9 / 60e-6 = 83.33 THz/s`

The transmitted signal in complex baseband:

```
s_TX(t) = exp(j * 2*pi * (fc*t + S/2 * t^2))
```

### 1.2 Beat Signal (Dechirp / Stretch Processing)

When the transmitted chirp reflects off a target at range R, the received signal is a delayed copy:

```
s_RX(t) = A * s_TX(t - tau)
```

Where `tau = 2*R/c` is the round-trip delay.

The AWR2243 mixes the received signal with the transmitted signal (dechirp):

```
s_IF(t) = s_TX(t) * conj(s_RX(t))
         = A * exp(j * 2*pi * (S*tau*t + fc*tau - S*tau^2/2))
```

This produces a beat signal with:

```
f_beat = S * tau = S * 2*R/c = (2 * BW * R) / (c * T_chirp)
```

And a constant phase term that encodes the precise range:

```
phi_0 = 2*pi*fc*tau = 4*pi*fc*R / c
```

**Key insight**: The beat frequency tells us the range, and the phase tells us sub-wavelength position changes. SAR exploits this phase information.

### 1.3 Range from Beat Frequency

Rearranging the beat frequency equation:

```
R = f_beat * c * T_chirp / (2 * BW)
```

For our system:
- Maximum IF bandwidth: 10 MHz (ADC sampling rate)
- Maximum beat frequency: 5 MHz (Nyquist)
- Maximum unambiguous range: `5e6 * 3e8 * 60e-6 / (2 * 5e9) = 9.0 m`


## 2. Range Resolution

The range resolution is determined solely by the chirp bandwidth:

```
delta_R = c / (2 * BW)
```

For our system:

```
delta_R = 3e8 / (2 * 5e9) = 0.03 m = 3 cm
```

This is the ability to distinguish two targets at different ranges. With windowing (Hanning), the -3 dB resolution broadens by a factor of ~1.6:

```
delta_R_windowed ~ 1.6 * c / (2 * BW) = 4.8 cm
```

But the sidelobe level improves from -13 dB (rectangular) to -31.5 dB (Hanning).

**Note**: Range resolution is independent of the antenna, scan geometry, or processing algorithm. It depends only on bandwidth.


## 3. SAR Principle

### 3.1 Physical vs. Synthetic Aperture

A physical antenna of length D has an angular resolution of approximately:

```
theta_3dB ~ lambda / D
```

At 77 GHz (`lambda = 3.9 mm`), a 2 cm antenna gives `theta ~ 11 degrees`, which translates to a cross-range resolution of:

```
delta_CR_real = R * theta_3dB = R * lambda / D
```

At R = 2m: `delta_CR = 2 * 0.0039 / 0.02 = 39 cm` -- very poor for imaging.

**SAR solution**: Move a small antenna along a path (the synthetic aperture) and coherently combine the received signals. This synthesizes an effective aperture much larger than the physical antenna.

### 3.2 Synthetic Aperture Formation

As the antenna moves along the x-axis, it collects signals at positions `x_n` for `n = 0, 1, ..., N-1`:

```
Position:  x_0     x_1     x_2     ...     x_{N-1}
           |       |       |               |
           v       v       v               v
    =============================================  <-- Synthetic aperture L
```

For a target at position (xt, yt), the range from antenna position x_n is:

```
R_n = sqrt((x_n - xt)^2 + yt^2)
```

The received phase at each position encodes this range:

```
phi_n = 4*pi*fc*R_n / c
```

By coherently combining all N measurements with proper phase compensation, we achieve the resolution of an antenna with physical length L (the synthetic aperture).

### 3.3 Cross-Range Resolution

For stripmap SAR with aperture length L at range R:

```
delta_CR = lambda * R / (2 * L)
```

For our system at different ranges:

| Range (R) | Aperture (L) | lambda | Cross-Range Resolution |
|---|---|---|---|
| 1.0 m | 30 cm | 3.9 mm | 0.65 cm |
| 2.0 m | 30 cm | 3.9 mm | 1.30 cm |
| 3.0 m | 30 cm | 3.9 mm | 1.95 cm |
| 5.0 m | 30 cm | 3.9 mm | 3.25 cm |

**Key insight**: Cross-range resolution degrades linearly with range (unlike range resolution, which is constant). At 77 GHz, even a 30 cm aperture provides sub-centimeter resolution at close range.

### 3.4 Nyquist Sampling in Cross-Range

To avoid aliasing (grating lobes) in the cross-range direction, the spatial sampling must satisfy:

```
delta_x <= lambda / 4
```

At 77 GHz: `delta_x <= 3.9 mm / 4 = 0.975 mm`

Our design uses 1 mm step size, which marginally satisfies the Nyquist criterion. For the MIMO virtual array with effective lambda/2 element spacing, the requirement is relaxed because the array already provides angular disambiguation.


## 4. Back-Projection Algorithm (BPA)

### 4.1 Algorithm Description

The Back-Projection Algorithm is the most flexible and intuitive SAR image formation method. It works by:

1. Define an image grid with pixels at positions (x_p, y_p)
2. For each pixel, calculate the range to each antenna position
3. Interpolate the range-compressed data at that range
4. Apply phase compensation for coherent summation
5. Sum contributions from all antenna positions

### 4.2 Mathematical Formulation

For each pixel (x_p, y_p):

```
I(x_p, y_p) = SUM_{n=0}^{N-1} [ S_rc(x_n, R_n) * exp(-j * 4*pi*fc*R_n / c) ]
```

Where:
- `S_rc(x_n, R_n)` = range-compressed data at position x_n, interpolated to range R_n
- `R_n = sqrt((x_n - x_p)^2 + y_p^2)` = slant range from antenna n to pixel
- `exp(-j * 4*pi*fc*R_n/c)` = phase compensation (matched filter)

### 4.3 Step-by-Step Implementation

```
1. RANGE COMPRESSION (for all positions):
   For each antenna position n:
     Apply window function to raw IF data
     Compute FFT along fast-time axis
     Store range-compressed profile S_rc[n, :]

2. IMAGE FORMATION (back-projection):
   Initialize image I[y, x] = 0 (complex)
   For each antenna position n:
     For each pixel (x_p, y_p):
       R = sqrt((x_n - x_p)^2 + y_p^2)
       bin_idx = R / range_bin_spacing
       S_interp = interpolate(S_rc[n, :], bin_idx)
       phase_comp = exp(-j * 4*pi*fc*R / c)
       I[y_p, x_p] += S_interp * phase_comp

3. OUTPUT:
   SAR image magnitude: |I(x, y)|
   SAR image phase: angle(I(x, y))
```

### 4.4 Computational Complexity

```
BPA complexity: O(Nx_img * Ny_img * N_positions)
```

For our system:
- Image grid: 150 x 600 = 90,000 pixels
- Antenna positions: 300
- Total operations: 90,000 x 300 = 27 million complex multiply-accumulate

Per operation: ~10 FLOPS (range calc, interpolation, phase, accumulate)
Total: ~270 MFLOPS per image

**On CPU** (single core, 3 GHz): ~0.5-1 second per image
**On GPU** (CUDA, RTX 3060): ~5-10 ms per image (massively parallel)

### 4.5 Advantages and Disadvantages

**Advantages:**
- Handles arbitrary scan geometries (linear, circular, freehand)
- No approximations -- exact focusing at all ranges
- Naturally handles near-field effects (important for short-range SAR)
- Easy to incorporate motion compensation
- Parallelizable on GPU

**Disadvantages:**
- Computationally expensive compared to FFT-based methods
- O(N^3) vs O(N^2 log N) for omega-k or chirp scaling
- Not suitable for real-time on FPGA without significant resources

For our handheld SAR (short range, small scenes, GPU available), BPA is the preferred algorithm.


## 5. Phase Gradient Autofocus (PGA)

### 5.1 Motivation

In freehand scanning mode, the IMU-estimated antenna positions will have errors on the order of millimeters. At 77 GHz (lambda = 3.9 mm), even 0.5 mm position error causes a phase error of:

```
phi_error = 4*pi * 0.5e-3 / 3.9e-3 = 1.61 radians (~92 degrees)
```

This is enough to completely defocus the SAR image. PGA corrects these residual phase errors after initial image formation.

### 5.2 Algorithm Overview

PGA is an iterative, data-driven autofocus technique that estimates and removes phase errors from the azimuth (slow-time) signal history.

### 5.3 Algorithm Steps

```
For iteration k = 1, 2, ..., K_max (typically 3-5):

  1. CIRCULAR SHIFT:
     For each range bin, find the dominant scatterer and
     circularly shift the data to center it at zero Doppler.

  2. WINDOWING:
     Apply a data-adaptive window centered on the dominant
     scatterer to isolate its phase history from nearby targets.

  3. PHASE GRADIENT ESTIMATION:
     Compute the phase gradient (derivative) of the windowed signal:
       g[n] = angle( SUM_m { S[m,n] * conj(S[m,n-1]) } )
     where the sum is over range bins m.

  4. PHASE ERROR ESTIMATION:
     Integrate the gradient to get the phase error:
       phi_err[n] = SUM_{k=0}^{n} g[k]
     Remove the linear component (which represents a simple
     position shift, not a focus error).

  5. PHASE CORRECTION:
     Multiply the range-compressed data by the conjugate of
     the estimated phase error:
       S_corrected[m,n] = S[m,n] * exp(-j * phi_err[n])

  6. RE-IMAGE:
     Apply back-projection (or FFT) with corrected data.

  Check convergence: if max|phi_err| < threshold, stop.
```

### 5.4 Convergence Properties

- **Typical convergence**: 3-5 iterations
- **Each iteration**: Removes the largest phase error component
- **Requires**: At least one strong point-like scatterer in the scene
- **Limitation**: Cannot correct range-dependent phase errors (needs more advanced methods like WPGA)

### 5.5 Implementation Considerations

For real-time operation:
- PGA adds ~50% overhead to BPA processing time
- Can be performed on GPU (same parallelism as BPA)
- For linear rail scans, PGA may not be needed (position is known)
- For freehand scans, PGA is essential


## 6. Motion Compensation

### 6.1 IMU-Based Position Estimation

The ICM-42688-P IMU provides:
- 3-axis accelerometer: +/- 16g, 16-bit, 1 kHz
- 3-axis gyroscope: +/- 2000 dps, 16-bit, 1 kHz

Position is estimated by double integration of acceleration:

```
v[k] = v[k-1] + a[k] * dt
x[k] = x[k-1] + v[k] * dt
```

### 6.2 Drift Problem

MEMS accelerometers have bias instability that causes quadratic position drift:

```
Position error ~ 0.5 * bias * t^2
```

For ICM-42688-P (bias instability ~25 ug):
- After 1 second: ~0.12 mm (acceptable)
- After 10 seconds: ~12 mm (too large for 77 GHz SAR)
- After 60 seconds: ~440 mm (completely unusable)

### 6.3 ZUPT (Zero Velocity Update)

ZUPT is a technique to reset velocity drift when the device is stationary:

```
1. Detect stationary periods:
   |a[k] - g| < threshold (e.g., 0.05 m/s^2)
   |omega[k]| < threshold (e.g., 0.1 rad/s)

2. When stationary:
   Set v[k] = 0
   Apply Kalman filter correction to position

3. Typical usage:
   Scan for 2-3 seconds, pause briefly, scan again
   ZUPT corrects accumulated drift at each pause
```

### 6.4 Phase Correction from Motion

Once antenna positions are estimated, the phase correction for each pulse is:

```
phi_correction[n] = -4*pi / lambda * delta_R[n]
```

Where `delta_R[n]` is the range error caused by position error at pulse n.

The corrected signal:

```
S_corrected[n, :] = S_raw[n, :] * exp(j * phi_correction[n])
```

### 6.5 Motion Compensation Pipeline

```
1. IMU data preprocessing:
   - Remove gravity vector (use gyro for orientation)
   - Apply low-pass filter (remove vibration)
   - Detect ZUPT intervals

2. Position estimation:
   - Strap-down integration with ZUPT corrections
   - Optional: Kalman filter fusion

3. Coarse motion compensation:
   - Apply range shift (integer range bins)
   - Apply phase correction (residual sub-bin range)

4. Fine autofocus (PGA):
   - Correct remaining errors iteratively
```


## 7. MIMO Virtual Array

### 7.1 MIMO Concept

The AWR2243 has 3 TX and 4 RX antennas. By transmitting from each TX sequentially (Time Division Multiplexing), we create a virtual array:

```
Physical TX positions: [0, 2d, 4d]     (3 elements, spacing 2d)
Physical RX positions: [0, d, 2d, 3d]  (4 elements, spacing d)

Virtual array = convolution of TX and RX positions:
  TX0 + RX: [0, d, 2d, 3d]
  TX1 + RX: [2d, 3d, 4d, 5d]
  TX2 + RX: [4d, 5d, 6d, 7d]

Virtual positions: [0, d, 2d, 3d, 4d, 5d, 6d, 7d, 4d, 5d, 6d, 7d]
After removing duplicates and sorting: [0, d, 2d, 3d, 4d, 5d, 6d, 7d]

Result: 8 unique virtual elements (or up to 12 with overlapping)
         with uniform d = lambda/2 spacing
```

### 7.2 Benefits for SAR

1. **Improved angular resolution**: 12 virtual elements vs 4 physical RX elements (3x improvement in angular resolution for real-aperture operation)

2. **Cross-range pre-filtering**: The MIMO array provides angular selectivity before SAR processing, reducing ambiguities and clutter

3. **Reduced scan density**: With better instantaneous angular resolution, the required spatial sampling rate can potentially be relaxed

4. **Phase center reconstruction**: Each virtual element has a well-defined phase center, simplifying the SAR geometry

### 7.3 TDM-MIMO Timing

```
Frame structure (one complete MIMO cycle):

TX0: |--chirp--|--idle--|
TX1:                     |--chirp--|--idle--|
TX2:                                         |--chirp--|--idle--|
     |<---------- Frame period = 3 * (T_chirp + T_idle) -------->|
     = 3 * (60us + 20us) = 240 us per frame
     Frame rate = 4167 frames/sec
```

### 7.4 Virtual Array SAR Processing

When using MIMO virtual array with SAR:

```
1. For each antenna position along the scan:
   a. Transmit from TX0, receive on all 4 RX -> 4 range profiles
   b. Transmit from TX1, receive on all 4 RX -> 4 range profiles
   c. Transmit from TX2, receive on all 4 RX -> 4 range profiles
   Total: 12 range profiles per position

2. Map each TX-RX combination to its virtual element position

3. Apply BPA using virtual element positions:
   - Each virtual element at each scan position contributes
   - Phase center = (TX_pos + RX_pos) / 2
   - Round-trip range = distance from TX to target + target to RX

4. Result: Enhanced SAR image with MIMO diversity
```


## 8. Resolution Summary

| Parameter | Value | Formula |
|---|---|---|
| Range resolution | 3.0 cm | c / (2*BW) |
| Range resolution (windowed) | ~4.8 cm | ~1.6 * c / (2*BW) |
| Cross-range resolution @ 1m | 0.65 cm | lambda*R / (2*L) |
| Cross-range resolution @ 2m | 1.30 cm | lambda*R / (2*L) |
| Cross-range resolution @ 3m | 1.95 cm | lambda*R / (2*L) |
| Cross-range resolution @ 5m | 3.25 cm | lambda*R / (2*L) |
| Angular resolution (MIMO, 12el) | ~3.3 deg | lambda / (N*d) |
| Maximum unambiguous range | 9.0 m | Fs*c*T / (2*BW) |
| Spatial Nyquist sampling | 0.975 mm | lambda / 4 |

## 9. Processing Pipeline Summary

```
         RAW ADC DATA
              |
    +---------v---------+
    |  Range Compression |  FFT along fast-time (per position)
    |  (fast-time FFT)   |  Windowed for sidelobe control
    +---------+----------+
              |
    +---------v---------+
    |  Motion            |  IMU integration + ZUPT
    |  Compensation      |  Phase correction per pulse
    +---------+----------+
              |
    +---------v---------+
    |  Back-Projection   |  Coherent summation over aperture
    |  Algorithm (BPA)   |  GPU-accelerated (CUDA)
    +---------+----------+
              |
    +---------v---------+
    |  Phase Gradient    |  Iterative residual phase correction
    |  Autofocus (PGA)   |  3-5 iterations typical
    +---------+----------+
              |
    +---------v---------+
    |  Post-Processing   |  Dynamic range compression
    |  & Display         |  Colormap, annotations, export
    +---------+----------+
              |
         SAR IMAGE
    (magnitude + phase)
```
