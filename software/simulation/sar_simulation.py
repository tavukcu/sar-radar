#!/usr/bin/env python3
"""
77 GHz Handheld SAR Radar - Complete SAR Simulation
====================================================

Simulates a 77 GHz FMCW radar performing a linear SAR scan.
Generates raw IF beat signals, applies range compression (FFT),
and forms a SAR image using the Back-Projection Algorithm (BPA).

No external radar libraries needed - uses only NumPy, SciPy, and Matplotlib.

Author: SAR Radar Project
Date: 2026-02-21
"""

import numpy as np
from scipy.signal import windows
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os
import time


# =============================================================================
# RADAR PARAMETERS
# =============================================================================

# Carrier frequency and bandwidth
fc = 78.5e9            # Center frequency [Hz]
BW = 5e9               # Chirp bandwidth [Hz]
c = 3e8                # Speed of light [m/s]
lam = c / fc           # Wavelength [m]

# Chirp parameters
T_chirp = 60e-6        # Chirp duration [s]
Fs = 10e6              # ADC sampling rate [Hz]
N_samples = 512        # Number of ADC samples per chirp
dt = 1.0 / Fs          # Sample interval [s]

# Derived parameters
slope = BW / T_chirp   # Chirp slope [Hz/s]
t_fast = np.arange(N_samples) * dt  # Fast-time axis [s]

# Range parameters
range_res = c / (2 * BW)                    # Range resolution [m]
max_beat = Fs / 2                            # Maximum beat frequency [Hz]
max_range = max_beat * c * T_chirp / (2 * BW)  # Maximum unambiguous range [m]

# SAR scan parameters
aperture_length = 0.30   # Synthetic aperture length [m]
step_size = 0.001        # Step size along x-axis [m]
N_positions = int(aperture_length / step_size)  # Number of antenna positions

# Antenna positions along x-axis (centered at origin)
x_ant = np.linspace(-aperture_length / 2, aperture_length / 2, N_positions)

# Cross-range resolution (theoretical)
# delta_cr = lambda * R / (2 * L) for stripmap SAR
# But with BPA and full aperture, delta_cr ~ lambda / (4 * arctan(L/(2*R)))


# =============================================================================
# TARGET SCENE DEFINITION
# =============================================================================

# 5 point targets at known positions: (x [m], y [m], amplitude)
targets = [
    (-0.05,  1.0, 1.0),   # Target 1: close range, left of center
    ( 0.05,  1.0, 1.0),   # Target 2: close range, right of center (test cross-range res)
    ( 0.00,  2.0, 1.0),   # Target 3: mid range, center
    (-0.08,  2.0, 0.8),   # Target 4: mid range, left
    ( 0.03,  3.0, 0.6),   # Target 5: far range, slightly right
]

print("=" * 70)
print("77 GHz FMCW SAR RADAR SIMULATION")
print("=" * 70)
print(f"\nRadar Parameters:")
print(f"  Center frequency:    {fc/1e9:.1f} GHz")
print(f"  Bandwidth:           {BW/1e9:.1f} GHz")
print(f"  Wavelength:          {lam*1e3:.2f} mm")
print(f"  Chirp duration:      {T_chirp*1e6:.0f} us")
print(f"  ADC sample rate:     {Fs/1e6:.0f} MHz")
print(f"  Samples per chirp:   {N_samples}")
print(f"  Chirp slope:         {slope/1e12:.2f} THz/s")
print(f"\nResolution:")
print(f"  Range resolution:    {range_res*100:.1f} cm")
print(f"  Max unambiguous range: {max_range:.2f} m")
print(f"\nSAR Scan Parameters:")
print(f"  Aperture length:     {aperture_length*100:.0f} cm")
print(f"  Step size:           {step_size*1e3:.1f} mm")
print(f"  Number of positions: {N_positions}")
print(f"\nTarget Scene:")
for i, (xt, yt, amp) in enumerate(targets):
    print(f"  Target {i+1}: x={xt*100:+6.1f} cm, y={yt*100:6.1f} cm, amp={amp:.1f}")


# =============================================================================
# SIGNAL GENERATION
# =============================================================================

print("\n--- Generating raw IF signals ---")
t_start = time.time()

# Raw data matrix: rows = antenna positions, cols = fast-time samples
raw_data = np.zeros((N_positions, N_samples), dtype=complex)

for pos_idx in range(N_positions):
    xa = x_ant[pos_idx]  # Current antenna position along x

    for xt, yt, amp in targets:
        # Slant range from antenna to target
        R = np.sqrt((xa - xt) ** 2 + yt ** 2)

        # Beat frequency for this target at this range
        # f_beat = 2 * slope * R / c  (round-trip delay * chirp slope)
        f_beat = 2 * slope * R / c

        # IF signal contribution from this target:
        #   - Beat tone at f_beat (from deramping/dechirp)
        #   - Phase term from round-trip propagation: exp(j * 4*pi*fc*R / c)
        phase_prop = 4.0 * np.pi * fc * R / c
        raw_data[pos_idx, :] += amp * np.exp(
            1j * 2 * np.pi * f_beat * t_fast
        ) * np.exp(1j * phase_prop)

# Add thermal noise (realistic SNR scenario)
noise_power = 0.01
noise = np.sqrt(noise_power / 2) * (
    np.random.randn(N_positions, N_samples)
    + 1j * np.random.randn(N_positions, N_samples)
)
raw_data += noise

t_gen = time.time() - t_start
print(f"  Signal generation complete in {t_gen:.2f} s")
print(f"  Raw data shape: {raw_data.shape}")
print(f"  Raw data size: {raw_data.nbytes / 1e6:.2f} MB")


# =============================================================================
# RANGE COMPRESSION (FFT along fast-time)
# =============================================================================

print("\n--- Range compression (fast-time FFT) ---")
t_start = time.time()

# Apply Hanning window along fast-time to reduce sidelobes
window = windows.hann(N_samples)
raw_windowed = raw_data * window[np.newaxis, :]

# FFT along fast-time (axis=1) for range compression
range_compressed = np.fft.fft(raw_windowed, axis=1)

# Only keep positive frequencies (one-sided spectrum)
N_range_bins = N_samples // 2
range_compressed = range_compressed[:, :N_range_bins]

# Range axis
freq_axis = np.fft.fftfreq(N_samples, d=dt)[:N_range_bins]
range_axis = freq_axis * c * T_chirp / (2 * BW)

t_rc = time.time() - t_start
print(f"  Range compression complete in {t_rc:.2f} s")
print(f"  Range-compressed data shape: {range_compressed.shape}")
print(f"  Range bin size: {(range_axis[1]-range_axis[0])*100:.2f} cm")


# =============================================================================
# BACK-PROJECTION ALGORITHM (SAR IMAGE FORMATION)
# =============================================================================

print("\n--- Back-Projection Algorithm ---")
t_start = time.time()

# Define image grid
# Cross-range (x) extent: slightly larger than aperture
x_img_min, x_img_max = -0.15, 0.15
# Range (y) extent: cover target range
y_img_min, y_img_max = 0.5, 3.5

# Pixel spacing (aim for sub-resolution sampling)
dx_img = 0.002   # 2 mm pixel spacing in cross-range
dy_img = 0.005   # 5 mm pixel spacing in range

x_img = np.arange(x_img_min, x_img_max, dx_img)
y_img = np.arange(y_img_min, y_img_max, dy_img)
Nx_img = len(x_img)
Ny_img = len(y_img)

print(f"  Image grid: {Nx_img} x {Ny_img} pixels")
print(f"  Cross-range extent: [{x_img_min*100:.0f}, {x_img_max*100:.0f}] cm")
print(f"  Range extent: [{y_img_min*100:.0f}, {y_img_max*100:.0f}] cm")

# Back-projection: for each pixel, coherently sum all range-compressed pulses
sar_image = np.zeros((Ny_img, Nx_img), dtype=complex)

# Range bin spacing for interpolation
range_bin_spacing = range_axis[1] - range_axis[0]

for pos_idx in range(N_positions):
    xa = x_ant[pos_idx]

    for xi in range(Nx_img):
        xp = x_img[xi]

        for yi in range(Ny_img):
            yp = y_img[yi]

            # Distance from antenna to pixel
            R_pixel = np.sqrt((xa - xp) ** 2 + yp ** 2)

            # Find corresponding range bin (fractional index)
            range_bin = R_pixel / range_bin_spacing

            # Bounds check
            if range_bin < 0 or range_bin >= N_range_bins - 1:
                continue

            # Linear interpolation of range-compressed data
            idx_low = int(np.floor(range_bin))
            idx_high = idx_low + 1
            frac = range_bin - idx_low

            if idx_high >= N_range_bins:
                continue

            rc_interp = (
                (1 - frac) * range_compressed[pos_idx, idx_low]
                + frac * range_compressed[pos_idx, idx_high]
            )

            # Phase compensation for coherent summation
            # Remove the propagation phase so all pulses add constructively
            phase_comp = np.exp(-1j * 4 * np.pi * fc * R_pixel / c)

            sar_image[yi, xi] += rc_interp * phase_comp

    # Progress indicator
    if (pos_idx + 1) % 50 == 0 or pos_idx == N_positions - 1:
        pct = 100.0 * (pos_idx + 1) / N_positions
        print(f"  Progress: {pos_idx+1}/{N_positions} positions ({pct:.0f}%)")

t_bpa = time.time() - t_start
print(f"  Back-projection complete in {t_bpa:.2f} s")


# =============================================================================
# RESOLUTION MEASUREMENT
# =============================================================================

print("\n--- Resolution Measurements ---")

sar_mag = np.abs(sar_image)
sar_db = 20 * np.log10(sar_mag / np.max(sar_mag) + 1e-10)


def measure_3db_width(data_1d, axis_1d):
    """Measure the -3 dB width of the main lobe in a 1D cut."""
    data_mag = np.abs(data_1d)
    peak_idx = np.argmax(data_mag)
    peak_val = data_mag[peak_idx]
    threshold = peak_val / np.sqrt(2)  # -3 dB

    # Find left edge
    left_idx = peak_idx
    while left_idx > 0 and data_mag[left_idx] > threshold:
        left_idx -= 1

    # Find right edge
    right_idx = peak_idx
    while right_idx < len(data_mag) - 1 and data_mag[right_idx] > threshold:
        right_idx += 1

    # Interpolate for better accuracy
    if left_idx > 0:
        frac_l = (threshold - data_mag[left_idx]) / (
            data_mag[left_idx + 1] - data_mag[left_idx] + 1e-20
        )
        x_left = axis_1d[left_idx] + frac_l * (axis_1d[left_idx + 1] - axis_1d[left_idx])
    else:
        x_left = axis_1d[0]

    if right_idx < len(data_mag) - 1:
        frac_r = (threshold - data_mag[right_idx]) / (
            data_mag[right_idx - 1] - data_mag[right_idx] + 1e-20
        )
        x_right = axis_1d[right_idx] - frac_r * (axis_1d[right_idx] - axis_1d[right_idx - 1])
    else:
        x_right = axis_1d[-1]

    return abs(x_right - x_left)


print(f"\n  {'Target':<10} {'Range[cm]':<12} {'X[cm]':<10} "
      f"{'Range Res[cm]':<16} {'X-Range Res[cm]':<16}")
print(f"  {'-'*64}")

for i, (xt, yt, amp) in enumerate(targets):
    # Find pixel closest to target
    xi_target = np.argmin(np.abs(x_img - xt))
    yi_target = np.argmin(np.abs(y_img - yt))

    # Measure range resolution (cut along y at target's x position)
    range_cut = sar_image[:, xi_target]
    range_res_meas = measure_3db_width(range_cut, y_img)

    # Measure cross-range resolution (cut along x at target's y position)
    xrange_cut = sar_image[yi_target, :]
    xrange_res_meas = measure_3db_width(xrange_cut, x_img)

    print(f"  T{i+1:<8} {yt*100:<12.1f} {xt*100:<+10.1f} "
          f"{range_res_meas*100:<16.2f} {xrange_res_meas*100:<16.2f}")

# Theoretical values
print(f"\n  Theoretical range resolution: {range_res*100:.2f} cm")
print(f"  Theoretical cross-range resolution at R=1m: "
      f"{lam * 1.0 / (2 * aperture_length) * 100:.2f} cm")
print(f"  Theoretical cross-range resolution at R=2m: "
      f"{lam * 2.0 / (2 * aperture_length) * 100:.2f} cm")
print(f"  Theoretical cross-range resolution at R=3m: "
      f"{lam * 3.0 / (2 * aperture_length) * 100:.2f} cm")
print(f"  (Note: Hanning window broadens main lobe by ~1.6x)")


# =============================================================================
# PLOTTING
# =============================================================================

print("\n--- Generating plots ---")

# Output directory
output_dir = os.path.dirname(os.path.abspath(__file__))

# ---------- Figure 1: Raw Data ----------
fig1, axes1 = plt.subplots(1, 2, figsize=(14, 5))
fig1.suptitle("Raw IF Data (Before Range Compression)", fontsize=14, fontweight="bold")

# Real part of raw data (2D image)
ax = axes1[0]
extent_raw = [0, N_samples, x_ant[-1] * 100, x_ant[0] * 100]
im = ax.imshow(
    np.real(raw_data),
    aspect="auto",
    cmap="RdBu",
    extent=extent_raw,
    interpolation="nearest",
)
ax.set_xlabel("Fast-time sample index")
ax.set_ylabel("Antenna position [cm]")
ax.set_title("Raw IF Signal (Real Part)")
plt.colorbar(im, ax=ax, label="Amplitude")

# Single chirp example
ax = axes1[1]
mid_pos = N_positions // 2
ax.plot(t_fast * 1e6, np.real(raw_data[mid_pos, :]), "b-", linewidth=0.5, label="Real")
ax.plot(t_fast * 1e6, np.imag(raw_data[mid_pos, :]), "r-", linewidth=0.5, alpha=0.6, label="Imag")
ax.set_xlabel("Fast time [us]")
ax.set_ylabel("Amplitude")
ax.set_title(f"Single Chirp IF Signal (Position #{mid_pos})")
ax.legend()
ax.grid(True, alpha=0.3)

fig1.tight_layout()
fig1.savefig(os.path.join(output_dir, "sar_raw_data.png"), dpi=150, bbox_inches="tight")
print(f"  Saved: sar_raw_data.png")

# ---------- Figure 2: Range-Compressed Data ----------
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle("Range-Compressed Data", fontsize=14, fontweight="bold")

# 2D range-compressed data
ax = axes2[0]
rc_mag_db = 20 * np.log10(np.abs(range_compressed) / np.max(np.abs(range_compressed)) + 1e-10)
extent_rc = [range_axis[0] * 100, range_axis[-1] * 100, x_ant[-1] * 100, x_ant[0] * 100]
im = ax.imshow(
    rc_mag_db,
    aspect="auto",
    cmap="jet",
    vmin=-40,
    vmax=0,
    extent=extent_rc,
    interpolation="nearest",
)
ax.set_xlabel("Range [cm]")
ax.set_ylabel("Antenna position [cm]")
ax.set_title("Range-Compressed Data (dB)")
plt.colorbar(im, ax=ax, label="dB")

# Mark target ranges
for i, (xt, yt, amp) in enumerate(targets):
    ax.axvline(x=yt * 100, color="w", linestyle="--", alpha=0.5, linewidth=0.8)

# Single range profile
ax = axes2[1]
ax.plot(range_axis * 100, 20 * np.log10(np.abs(range_compressed[mid_pos, :]) + 1e-10), "b-")
ax.set_xlabel("Range [cm]")
ax.set_ylabel("Magnitude [dB]")
ax.set_title(f"Range Profile (Position #{mid_pos})")
ax.grid(True, alpha=0.3)

# Mark target ranges
for i, (xt, yt, amp) in enumerate(targets):
    ax.axvline(x=yt * 100, color="r", linestyle="--", alpha=0.5, linewidth=0.8,
               label=f"T{i+1}: {yt*100:.0f}cm" if i == 0 else f"T{i+1}: {yt*100:.0f}cm")
ax.legend(fontsize=8)

fig2.tight_layout()
fig2.savefig(os.path.join(output_dir, "sar_range_compressed.png"), dpi=150, bbox_inches="tight")
print(f"  Saved: sar_range_compressed.png")

# ---------- Figure 3: SAR Image ----------
fig3 = plt.figure(figsize=(16, 10))
gs = gridspec.GridSpec(2, 3, figure=fig3, width_ratios=[1, 1, 0.8])
fig3.suptitle("SAR Image (Back-Projection Algorithm)", fontsize=14, fontweight="bold")

# SAR image (dB scale)
ax1 = fig3.add_subplot(gs[0, 0])
extent_img = [x_img[0] * 100, x_img[-1] * 100, y_img[-1] * 100, y_img[0] * 100]
im1 = ax1.imshow(
    sar_db,
    aspect="auto",
    cmap="hot",
    vmin=-30,
    vmax=0,
    extent=extent_img,
    interpolation="nearest",
)
ax1.set_xlabel("Cross-range [cm]")
ax1.set_ylabel("Range [cm]")
ax1.set_title("SAR Image (dB)")
plt.colorbar(im1, ax=ax1, label="dB")

# Mark true target positions
for i, (xt, yt, amp) in enumerate(targets):
    ax1.plot(xt * 100, yt * 100, "c+", markersize=10, markeredgewidth=2)
    ax1.annotate(
        f"T{i+1}", (xt * 100, yt * 100),
        textcoords="offset points", xytext=(8, 5),
        fontsize=8, color="cyan"
    )

# SAR image (linear scale)
ax2 = fig3.add_subplot(gs[0, 1])
im2 = ax2.imshow(
    sar_mag / np.max(sar_mag),
    aspect="auto",
    cmap="gray",
    vmin=0,
    vmax=1,
    extent=extent_img,
    interpolation="nearest",
)
ax2.set_xlabel("Cross-range [cm]")
ax2.set_ylabel("Range [cm]")
ax2.set_title("SAR Image (Linear)")
plt.colorbar(im2, ax=ax2, label="Normalized")

for i, (xt, yt, amp) in enumerate(targets):
    ax2.plot(xt * 100, yt * 100, "r+", markersize=10, markeredgewidth=2)

# Range cut through Target 1 (y = 1m)
ax3 = fig3.add_subplot(gs[0, 2])
yi_cut = np.argmin(np.abs(y_img - 1.0))
xrange_cut = sar_image[yi_cut, :]
xrange_cut_db = 20 * np.log10(np.abs(xrange_cut) / np.max(np.abs(xrange_cut)) + 1e-10)
ax3.plot(x_img * 100, xrange_cut_db, "b-")
ax3.set_xlabel("Cross-range [cm]")
ax3.set_ylabel("Magnitude [dB]")
ax3.set_title("Cross-range Cut at R=1m")
ax3.set_ylim(-40, 5)
ax3.axhline(y=-3, color="r", linestyle="--", alpha=0.5, label="-3 dB")
ax3.grid(True, alpha=0.3)
ax3.legend(fontsize=8)

# Cross-range cut through Target 3 (x = 0)
ax4 = fig3.add_subplot(gs[1, 0])
xi_cut = np.argmin(np.abs(x_img - 0.0))
range_cut = sar_image[:, xi_cut]
range_cut_db = 20 * np.log10(np.abs(range_cut) / np.max(np.abs(range_cut)) + 1e-10)
ax4.plot(y_img * 100, range_cut_db, "b-")
ax4.set_xlabel("Range [cm]")
ax4.set_ylabel("Magnitude [dB]")
ax4.set_title("Range Cut at x=0 cm")
ax4.set_ylim(-40, 5)
ax4.axhline(y=-3, color="r", linestyle="--", alpha=0.5, label="-3 dB")
ax4.grid(True, alpha=0.3)
ax4.legend(fontsize=8)

# Phase of SAR image
ax5 = fig3.add_subplot(gs[1, 1])
# Only show phase where signal is strong enough
phase_img = np.angle(sar_image)
phase_masked = np.where(sar_db > -20, phase_img, np.nan)
im5 = ax5.imshow(
    phase_masked,
    aspect="auto",
    cmap="hsv",
    vmin=-np.pi,
    vmax=np.pi,
    extent=extent_img,
    interpolation="nearest",
)
ax5.set_xlabel("Cross-range [cm]")
ax5.set_ylabel("Range [cm]")
ax5.set_title("SAR Phase (where > -20 dB)")
plt.colorbar(im5, ax=ax5, label="Phase [rad]")

# Summary text
ax6 = fig3.add_subplot(gs[1, 2])
ax6.axis("off")
summary_text = (
    f"Simulation Parameters\n"
    f"{'='*28}\n"
    f"fc = {fc/1e9:.1f} GHz\n"
    f"BW = {BW/1e9:.1f} GHz\n"
    f"lambda = {lam*1e3:.2f} mm\n"
    f"T_chirp = {T_chirp*1e6:.0f} us\n"
    f"Fs = {Fs/1e6:.0f} MHz\n"
    f"N_samples = {N_samples}\n"
    f"\n"
    f"Aperture = {aperture_length*100:.0f} cm\n"
    f"Step = {step_size*1e3:.1f} mm\n"
    f"Positions = {N_positions}\n"
    f"\n"
    f"Range Res = {range_res*100:.1f} cm\n"
    f"X-Res @1m = {lam*1.0/(2*aperture_length)*100:.2f} cm\n"
    f"X-Res @2m = {lam*2.0/(2*aperture_length)*100:.2f} cm\n"
    f"X-Res @3m = {lam*3.0/(2*aperture_length)*100:.2f} cm\n"
    f"\n"
    f"Processing time:\n"
    f"  Signal gen: {t_gen:.1f} s\n"
    f"  Range FFT: {t_rc:.2f} s\n"
    f"  BPA: {t_bpa:.1f} s\n"
)
ax6.text(
    0.05, 0.95, summary_text, transform=ax6.transAxes,
    fontsize=9, verticalalignment="top", fontfamily="monospace",
    bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8),
)

fig3.tight_layout()
fig3.savefig(os.path.join(output_dir, "sar_image.png"), dpi=150, bbox_inches="tight")
print(f"  Saved: sar_image.png")

# ---------- Figure 4: Point Spread Function Analysis ----------
fig4, axes4 = plt.subplots(1, 3, figsize=(16, 5))
fig4.suptitle("Point Spread Function (PSF) Analysis", fontsize=14, fontweight="bold")

# PSF at R = 1 m (Target 1)
for ax_idx, (target_idx, label) in enumerate([(0, "T1 (R=1m)"), (2, "T3 (R=2m)"), (4, "T5 (R=3m)")]):
    ax = axes4[ax_idx]
    xt, yt, amp = targets[target_idx]

    yi_t = np.argmin(np.abs(y_img - yt))
    xi_t = np.argmin(np.abs(x_img - xt))

    # Extract PSF region (cross-range cut)
    xr_cut = np.abs(sar_image[yi_t, :])
    xr_cut_norm = xr_cut / np.max(xr_cut)
    xr_cut_db = 20 * np.log10(xr_cut_norm + 1e-10)

    ax.plot((x_img - xt) * 100, xr_cut_db, "b-", linewidth=1.5)
    ax.axhline(y=-3, color="r", linestyle="--", alpha=0.7, label="-3 dB")
    ax.axhline(y=-13.2, color="g", linestyle="--", alpha=0.5, label="-13.2 dB (first null Hann)")
    ax.set_xlabel("Cross-range offset [cm]")
    ax.set_ylabel("Magnitude [dB]")
    ax.set_title(f"PSF Cross-range: {label}")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-40, 5)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)

fig4.tight_layout()
fig4.savefig(os.path.join(output_dir, "sar_psf_analysis.png"), dpi=150, bbox_inches="tight")
print(f"  Saved: sar_psf_analysis.png")

plt.show()

print("\n" + "=" * 70)
print("SIMULATION COMPLETE")
print("=" * 70)
