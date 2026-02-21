#!/usr/bin/env python3
"""
77 GHz Patch Antenna and MIMO Array Pattern Simulation
=======================================================

Calculates radiation patterns for:
1. Single rectangular microstrip patch antenna at 77 GHz
2. 3-element TX array
3. 4-element RX array
4. 12-element MIMO virtual array

Substrate: Rogers RO4350B (er=3.66, h=0.254 mm, tan_d=0.0037)

Author: SAR Radar Project
Date: 2026-02-21
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import os


# =============================================================================
# CONSTANTS AND SUBSTRATE PARAMETERS
# =============================================================================

c = 3e8                    # Speed of light [m/s]
f0 = 77e9                  # Center frequency [Hz]
lam0 = c / f0              # Free-space wavelength [m]
k0 = 2 * np.pi / lam0     # Free-space wavenumber [1/m]

# Rogers RO4350B substrate
er = 3.66                  # Relative permittivity
h = 0.254e-3               # Substrate thickness [m]
tan_d = 0.0037             # Loss tangent

print("=" * 70)
print("77 GHz PATCH ANTENNA AND MIMO ARRAY SIMULATION")
print("=" * 70)
print(f"\nFrequency: {f0/1e9:.0f} GHz")
print(f"Free-space wavelength: {lam0*1e3:.3f} mm")
print(f"Substrate: Rogers RO4350B")
print(f"  er = {er}, h = {h*1e3:.3f} mm, tan_d = {tan_d}")


# =============================================================================
# PATCH ANTENNA DIMENSIONS
# =============================================================================

# Effective dielectric constant (microstrip approximation)
er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 + 12 * h / (lam0 / 2)) ** (-0.5)
# More accurate: use W for the calculation (iterate)

# Patch width (for efficient radiation, W ~ c/(2*f0) * sqrt(2/(er+1)))
W = c / (2 * f0) * np.sqrt(2 / (er + 1))

# Recalculate effective dielectric constant with actual width
W_over_h = W / h
if W_over_h > 1:
    er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 / np.sqrt(1 + 12 * h / W))
else:
    er_eff = (er + 1) / 2 + (er - 1) / 2 * (
        (1 / np.sqrt(1 + 12 * h / W)) + 0.04 * (1 - W / h) ** 2
    )

# Wavelength in substrate
lam_eff = lam0 / np.sqrt(er_eff)

# Fringing extension (Hammerstad formula)
delta_L = 0.412 * h * (
    (er_eff + 0.3) * (W / h + 0.264)
    / ((er_eff - 0.258) * (W / h + 0.8))
)

# Patch length (resonant at f0)
L = c / (2 * f0 * np.sqrt(er_eff)) - 2 * delta_L

# Ground plane dimensions (finite ground)
Lg = L + 6 * h
Wg = W + 6 * h

# 50-ohm microstrip feed line width (approximate formula)
# For RO4350B at 77 GHz
A_ms = (60.0 / np.sqrt(er)) * np.log(8 * h / W + W / (4 * h)) if W / h < 1 else 0
B_ms = 377 * np.pi / (2 * np.sqrt(er))
# Iterative would be better, but approximate:
W_50ohm = 7.475 * h * np.exp(-50 * np.sqrt(er + 1.41) / 87) - 1.25 * (2 * h)
if W_50ohm < 0:
    W_50ohm = 0.15e-3  # Fallback to typical value

print(f"\nPatch Antenna Dimensions:")
print(f"  Patch width  (W): {W*1e3:.3f} mm")
print(f"  Patch length (L): {L*1e3:.3f} mm")
print(f"  Effective er:      {er_eff:.3f}")
print(f"  Lambda_eff:        {lam_eff*1e3:.3f} mm")
print(f"  Fringing dL:       {delta_L*1e3:.4f} mm")
print(f"  Ground plane:      {Lg*1e3:.3f} x {Wg*1e3:.3f} mm")
print(f"  50-ohm line width: {W_50ohm*1e3:.3f} mm")


# =============================================================================
# SINGLE ELEMENT RADIATION PATTERN
# =============================================================================

def patch_element_pattern(theta, phi, W, L, h, er_eff, k0):
    """
    Calculate the radiation pattern of a rectangular microstrip patch antenna.

    Uses the cavity model with two radiating slots.

    Parameters
    ----------
    theta : ndarray - Elevation angle [rad] (0 = broadside)
    phi : ndarray - Azimuth angle [rad] (0 = E-plane)
    W : float - Patch width [m]
    L : float - Patch length [m]
    h : float - Substrate height [m]
    er_eff : float - Effective dielectric constant
    k0 : float - Free-space wavenumber [1/m]

    Returns
    -------
    E_total : ndarray - Normalized electric field magnitude
    """
    # Radiating slot model (two slots separated by L_eff)
    L_eff = L + 2 * 0.412 * h * (
        (er_eff + 0.3) * (W / h + 0.264) / ((er_eff - 0.258) * (W / h + 0.8))
    )

    # E-plane pattern (phi = 0): due to two slots
    # Slot factor (single slot of width W, uniform aperture)
    kx = k0 * np.sin(theta) * np.cos(phi)
    ky = k0 * np.sin(theta) * np.sin(phi)

    # Slot radiation: sinc pattern from slot width
    arg_W = k0 * W * np.sin(theta) * np.sin(phi) / 2
    slot_W = np.where(np.abs(arg_W) < 1e-10, 1.0, np.sin(arg_W) / arg_W)

    # Slot height factor (thin slot of height h)
    arg_h = k0 * h * np.sin(theta) * np.cos(phi) / 2
    slot_h = np.where(np.abs(arg_h) < 1e-10, 1.0, np.sin(arg_h) / arg_h)

    # Array factor of two slots separated by L_eff (along x for E-plane)
    psi = k0 * L_eff * np.sin(theta) * np.cos(phi)
    AF_slots = np.cos(psi / 2)

    # Combined element pattern
    E_total = np.abs(slot_W * slot_h * AF_slots * np.cos(theta))

    return E_total


# Compute E-plane and H-plane patterns
theta_1d = np.linspace(-np.pi / 2, np.pi / 2, 361)

# E-plane: phi = 0 (xz-plane, along patch length)
E_plane = patch_element_pattern(theta_1d, np.zeros_like(theta_1d), W, L, h, er_eff, k0)
E_plane_norm = E_plane / np.max(E_plane + 1e-20)
E_plane_dB = 20 * np.log10(E_plane_norm + 1e-10)

# H-plane: phi = pi/2 (yz-plane, along patch width)
H_plane = patch_element_pattern(
    theta_1d, np.full_like(theta_1d, np.pi / 2), W, L, h, er_eff, k0
)
H_plane_norm = H_plane / np.max(H_plane + 1e-20)
H_plane_dB = 20 * np.log10(H_plane_norm + 1e-10)


# Element beamwidth
def find_beamwidth(pattern_db, theta_axis):
    """Find -3 dB beamwidth."""
    peak_idx = np.argmax(pattern_db)
    left = peak_idx
    while left > 0 and pattern_db[left] > -3:
        left -= 1
    right = peak_idx
    while right < len(pattern_db) - 1 and pattern_db[right] > -3:
        right += 1
    return np.degrees(theta_axis[right] - theta_axis[left])


bw_e = find_beamwidth(E_plane_dB, theta_1d)
bw_h = find_beamwidth(H_plane_dB, theta_1d)

# Element directivity estimate (approximate)
# D ~ 4*pi / (theta_E * theta_H) in steradians
D_element = 4 * np.pi / (np.radians(bw_e) * np.radians(bw_h))
G_element_dBi = 10 * np.log10(D_element)

print(f"\nSingle Element Pattern:")
print(f"  E-plane beamwidth (-3dB): {bw_e:.1f} deg")
print(f"  H-plane beamwidth (-3dB): {bw_h:.1f} deg")
print(f"  Estimated directivity:    {G_element_dBi:.1f} dBi")


# =============================================================================
# ARRAY CONFIGURATIONS
# =============================================================================

# Element spacing (lambda/2 in free space for grating-lobe-free operation)
d_element = lam0 / 2  # ~1.95 mm

# TX array: 3 elements, spaced 2*d apart (for MIMO virtual array)
N_tx = 3
tx_positions = np.arange(N_tx) * 2 * d_element  # [0, 2d, 4d]
tx_positions -= np.mean(tx_positions)  # Center the array

# RX array: 4 elements, spaced d apart
N_rx = 4
rx_positions = np.arange(N_rx) * d_element  # [0, d, 2d, 3d]
rx_positions -= np.mean(rx_positions)  # Center the array

# Virtual array (MIMO): convolution of TX and RX positions
# Results in 12 virtual elements with lambda/2 spacing
virtual_positions = []
for tx_pos in tx_positions:
    for rx_pos in rx_positions:
        virtual_positions.append(tx_pos + rx_pos)
virtual_positions = np.array(sorted(virtual_positions))
# Normalize to be centered
virtual_positions -= np.mean(virtual_positions)
N_virtual = len(virtual_positions)

print(f"\nArray Configurations:")
print(f"  Element spacing d = lambda/2 = {d_element*1e3:.3f} mm")
print(f"  TX array: {N_tx} elements, spacing = 2d = {2*d_element*1e3:.3f} mm")
print(f"    Positions: {tx_positions*1e3} mm")
print(f"  RX array: {N_rx} elements, spacing = d = {d_element*1e3:.3f} mm")
print(f"    Positions: {rx_positions*1e3} mm")
print(f"  Virtual (MIMO) array: {N_virtual} elements")
print(f"    Positions: {np.round(virtual_positions*1e3, 3)} mm")


def array_factor(theta, positions, k0, steering_angle=0):
    """
    Calculate the array factor for a linear array.

    Parameters
    ----------
    theta : ndarray - Angle from broadside [rad]
    positions : ndarray - Element positions [m]
    k0 : float - Wavenumber [1/m]
    steering_angle : float - Beam steering angle [rad]

    Returns
    -------
    AF : ndarray - Normalized array factor
    """
    AF = np.zeros(len(theta), dtype=complex)
    for pos in positions:
        phase = k0 * pos * (np.sin(theta) - np.sin(steering_angle))
        AF += np.exp(1j * phase)

    AF_mag = np.abs(AF) / len(positions)  # Normalize by number of elements
    return AF_mag


# Compute array factors
theta_fine = np.linspace(-np.pi / 2, np.pi / 2, 1801)

AF_tx = array_factor(theta_fine, tx_positions, k0)
AF_rx = array_factor(theta_fine, rx_positions, k0)
AF_virtual = array_factor(theta_fine, virtual_positions, k0)

AF_tx_dB = 20 * np.log10(AF_tx + 1e-10)
AF_rx_dB = 20 * np.log10(AF_rx + 1e-10)
AF_virtual_dB = 20 * np.log10(AF_virtual + 1e-10)

# Combined patterns (element * array factor)
E_element_fine = patch_element_pattern(
    theta_fine, np.zeros_like(theta_fine), W, L, h, er_eff, k0
)
E_element_fine_norm = E_element_fine / np.max(E_element_fine + 1e-20)

total_tx = E_element_fine_norm * AF_tx
total_rx = E_element_fine_norm * AF_rx
total_virtual = E_element_fine_norm * AF_virtual

total_tx_dB = 20 * np.log10(total_tx / np.max(total_tx + 1e-20) + 1e-10)
total_rx_dB = 20 * np.log10(total_rx / np.max(total_rx + 1e-20) + 1e-10)
total_virtual_dB = 20 * np.log10(total_virtual / np.max(total_virtual + 1e-20) + 1e-10)

# Beamwidths
bw_tx = find_beamwidth(total_tx_dB, theta_fine)
bw_rx = find_beamwidth(total_rx_dB, theta_fine)
bw_virtual = find_beamwidth(total_virtual_dB, theta_fine)

# Array gains (approximate)
G_tx_dBi = G_element_dBi + 10 * np.log10(N_tx)
G_rx_dBi = G_element_dBi + 10 * np.log10(N_rx)
G_virtual_dBi = G_element_dBi + 10 * np.log10(N_virtual)

print(f"\nArray Beamwidths and Gains:")
print(f"  TX array ({N_tx} el):      BW = {bw_tx:.1f} deg, Gain ~ {G_tx_dBi:.1f} dBi")
print(f"  RX array ({N_rx} el):      BW = {bw_rx:.1f} deg, Gain ~ {G_rx_dBi:.1f} dBi")
print(f"  Virtual MIMO ({N_virtual} el): BW = {bw_virtual:.1f} deg, Gain ~ {G_virtual_dBi:.1f} dBi")


# =============================================================================
# PLOTTING
# =============================================================================

output_dir = os.path.dirname(os.path.abspath(__file__))

# ---------- Figure 1: Single Element Patterns ----------
fig1, axes1 = plt.subplots(1, 3, figsize=(16, 5))
fig1.suptitle(
    f"Single Patch Element Pattern at {f0/1e9:.0f} GHz "
    f"(W={W*1e3:.2f}mm, L={L*1e3:.2f}mm)",
    fontsize=13, fontweight="bold"
)

# Rectangular plot - E and H planes
ax = axes1[0]
ax.plot(np.degrees(theta_1d), E_plane_dB, "b-", linewidth=2, label=f"E-plane (BW={bw_e:.1f})")
ax.plot(np.degrees(theta_1d), H_plane_dB, "r--", linewidth=2, label=f"H-plane (BW={bw_h:.1f})")
ax.axhline(y=-3, color="gray", linestyle=":", alpha=0.5)
ax.set_xlabel("Theta [deg]")
ax.set_ylabel("Normalized Pattern [dB]")
ax.set_title("E-plane and H-plane")
ax.set_xlim(-90, 90)
ax.set_ylim(-30, 3)
ax.grid(True, alpha=0.3)
ax.legend()

# Polar plot - E-plane
ax = axes1[1]
ax_polar = fig1.add_axes(ax.get_position(), projection="polar")
ax.set_visible(False)
ax_polar.plot(theta_1d + np.pi / 2, E_plane_dB + 30, "b-", linewidth=1.5)
ax_polar.set_rmax(35)
ax_polar.set_rmin(0)
ax_polar.set_rticks([0, 10, 20, 30])
ax_polar.set_rlabel_position(135)
ax_polar.set_title("E-plane (Polar)", pad=20)
ax_polar.set_theta_zero_location("N")

# Polar plot - H-plane
ax = axes1[2]
ax_polar2 = fig1.add_axes(ax.get_position(), projection="polar")
ax.set_visible(False)
ax_polar2.plot(theta_1d + np.pi / 2, H_plane_dB + 30, "r-", linewidth=1.5)
ax_polar2.set_rmax(35)
ax_polar2.set_rmin(0)
ax_polar2.set_rticks([0, 10, 20, 30])
ax_polar2.set_rlabel_position(135)
ax_polar2.set_title("H-plane (Polar)", pad=20)
ax_polar2.set_theta_zero_location("N")

fig1.savefig(
    os.path.join(output_dir, "antenna_single_element.png"),
    dpi=150, bbox_inches="tight"
)
print(f"\nSaved: antenna_single_element.png")

# ---------- Figure 2: Array Factor Comparison ----------
fig2, axes2 = plt.subplots(2, 2, figsize=(14, 10))
fig2.suptitle("Array Factor Comparison", fontsize=14, fontweight="bold")

# TX array factor
ax = axes2[0, 0]
ax.plot(np.degrees(theta_fine), AF_tx_dB, "b-", linewidth=1.5)
ax.set_xlabel("Theta [deg]")
ax.set_ylabel("Array Factor [dB]")
ax.set_title(f"TX Array Factor ({N_tx} elements, spacing=2d)")
ax.set_xlim(-90, 90)
ax.set_ylim(-30, 3)
ax.axhline(y=-3, color="gray", linestyle=":", alpha=0.5)
ax.grid(True, alpha=0.3)

# RX array factor
ax = axes2[0, 1]
ax.plot(np.degrees(theta_fine), AF_rx_dB, "r-", linewidth=1.5)
ax.set_xlabel("Theta [deg]")
ax.set_ylabel("Array Factor [dB]")
ax.set_title(f"RX Array Factor ({N_rx} elements, spacing=d)")
ax.set_xlim(-90, 90)
ax.set_ylim(-30, 3)
ax.axhline(y=-3, color="gray", linestyle=":", alpha=0.5)
ax.grid(True, alpha=0.3)

# Virtual (MIMO) array factor
ax = axes2[1, 0]
ax.plot(np.degrees(theta_fine), AF_virtual_dB, "g-", linewidth=1.5)
ax.set_xlabel("Theta [deg]")
ax.set_ylabel("Array Factor [dB]")
ax.set_title(f"Virtual MIMO Array Factor ({N_virtual} elements)")
ax.set_xlim(-90, 90)
ax.set_ylim(-30, 3)
ax.axhline(y=-3, color="gray", linestyle=":", alpha=0.5)
ax.grid(True, alpha=0.3)

# All combined patterns
ax = axes2[1, 1]
ax.plot(np.degrees(theta_fine), total_tx_dB, "b-", linewidth=1.5,
        label=f"TX ({N_tx}el, BW={bw_tx:.1f})")
ax.plot(np.degrees(theta_fine), total_rx_dB, "r--", linewidth=1.5,
        label=f"RX ({N_rx}el, BW={bw_rx:.1f})")
ax.plot(np.degrees(theta_fine), total_virtual_dB, "g-.", linewidth=2,
        label=f"Virtual ({N_virtual}el, BW={bw_virtual:.1f})")
ax.set_xlabel("Theta [deg]")
ax.set_ylabel("Total Pattern [dB]")
ax.set_title("Combined Patterns (Element x AF)")
ax.set_xlim(-90, 90)
ax.set_ylim(-30, 3)
ax.axhline(y=-3, color="gray", linestyle=":", alpha=0.5)
ax.grid(True, alpha=0.3)
ax.legend(fontsize=9)

fig2.tight_layout()
fig2.savefig(
    os.path.join(output_dir, "antenna_array_patterns.png"),
    dpi=150, bbox_inches="tight"
)
print(f"Saved: antenna_array_patterns.png")

# ---------- Figure 3: Array Element Positions ----------
fig3, axes3 = plt.subplots(3, 1, figsize=(12, 8))
fig3.suptitle("Array Element Positions", fontsize=14, fontweight="bold")

# TX positions
ax = axes3[0]
ax.stem(tx_positions * 1e3, np.ones(N_tx), linefmt="b-", markerfmt="bo", basefmt="k-")
ax.set_xlabel("Position [mm]")
ax.set_ylabel("Amplitude")
ax.set_title(f"TX Array: {N_tx} elements (spacing = 2d = {2*d_element*1e3:.2f} mm)")
ax.set_ylim(0, 1.5)
ax.grid(True, alpha=0.3)
for i, pos in enumerate(tx_positions):
    ax.annotate(f"TX{i}", (pos * 1e3, 1.05), ha="center", fontsize=9, color="blue")

# RX positions
ax = axes3[1]
ax.stem(rx_positions * 1e3, np.ones(N_rx), linefmt="r-", markerfmt="ro", basefmt="k-")
ax.set_xlabel("Position [mm]")
ax.set_ylabel("Amplitude")
ax.set_title(f"RX Array: {N_rx} elements (spacing = d = {d_element*1e3:.2f} mm)")
ax.set_ylim(0, 1.5)
ax.grid(True, alpha=0.3)
for i, pos in enumerate(rx_positions):
    ax.annotate(f"RX{i}", (pos * 1e3, 1.05), ha="center", fontsize=9, color="red")

# Virtual array positions
ax = axes3[2]
ax.stem(
    virtual_positions * 1e3, np.ones(N_virtual),
    linefmt="g-", markerfmt="g^", basefmt="k-"
)
ax.set_xlabel("Position [mm]")
ax.set_ylabel("Amplitude")
ax.set_title(f"Virtual MIMO Array: {N_virtual} elements (effective spacing = d/1)")
ax.set_ylim(0, 1.5)
ax.grid(True, alpha=0.3)
for i, pos in enumerate(virtual_positions):
    ax.annotate(f"V{i}", (pos * 1e3, 1.05), ha="center", fontsize=7, color="green")

fig3.tight_layout()
fig3.savefig(
    os.path.join(output_dir, "antenna_array_positions.png"),
    dpi=150, bbox_inches="tight"
)
print(f"Saved: antenna_array_positions.png")

# ---------- Figure 4: 3D Radiation Pattern ----------
fig4 = plt.figure(figsize=(14, 6))

theta_3d = np.linspace(0, np.pi / 2, 91)     # 0 to 90 deg elevation
phi_3d = np.linspace(0, 2 * np.pi, 361)       # 0 to 360 deg azimuth
THETA, PHI = np.meshgrid(theta_3d, phi_3d)

# Compute 3D element pattern
E_3d = patch_element_pattern(THETA, PHI, W, L, h, er_eff, k0)
E_3d_norm = E_3d / np.max(E_3d + 1e-20)
E_3d_dB = 20 * np.log10(E_3d_norm + 1e-10)
E_3d_dB_clipped = np.clip(E_3d_dB, -30, 0)

# Convert to Cartesian for 3D plot
R_plot = E_3d_dB_clipped + 30  # Shift so -30 dB maps to 0
X_3d = R_plot * np.sin(THETA) * np.cos(PHI)
Y_3d = R_plot * np.sin(THETA) * np.sin(PHI)
Z_3d = R_plot * np.cos(THETA)

# Single element 3D
ax1 = fig4.add_subplot(121, projection="3d")
surf1 = ax1.plot_surface(
    X_3d, Y_3d, Z_3d,
    facecolors=cm.jet((E_3d_dB_clipped + 30) / 30),
    alpha=0.8, linewidth=0, antialiased=True
)
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.set_title("Single Element 3D Pattern", fontsize=11)
ax1.view_init(elev=25, azim=45)

# Virtual array 3D pattern (in H-plane / azimuth, modulated by element)
# For the virtual array, array factor depends on phi (azimuth scan)
AF_3d = np.zeros_like(THETA, dtype=complex)
for pos in virtual_positions:
    AF_3d += np.exp(1j * k0 * pos * np.sin(THETA) * np.sin(PHI))
AF_3d_mag = np.abs(AF_3d) / N_virtual

# Combined
Total_3d = E_3d_norm * AF_3d_mag
Total_3d_norm = Total_3d / np.max(Total_3d + 1e-20)
Total_3d_dB = 20 * np.log10(Total_3d_norm + 1e-10)
Total_3d_dB_clipped = np.clip(Total_3d_dB, -30, 0)

R_plot2 = Total_3d_dB_clipped + 30
X_3d2 = R_plot2 * np.sin(THETA) * np.cos(PHI)
Y_3d2 = R_plot2 * np.sin(THETA) * np.sin(PHI)
Z_3d2 = R_plot2 * np.cos(THETA)

ax2 = fig4.add_subplot(122, projection="3d")
surf2 = ax2.plot_surface(
    X_3d2, Y_3d2, Z_3d2,
    facecolors=cm.jet((Total_3d_dB_clipped + 30) / 30),
    alpha=0.8, linewidth=0, antialiased=True
)
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")
ax2.set_title(f"MIMO Virtual Array ({N_virtual}el) 3D Pattern", fontsize=11)
ax2.view_init(elev=25, azim=45)

fig4.suptitle("3D Radiation Patterns at 77 GHz", fontsize=14, fontweight="bold")
fig4.tight_layout()
fig4.savefig(
    os.path.join(output_dir, "antenna_3d_pattern.png"),
    dpi=150, bbox_inches="tight"
)
print(f"Saved: antenna_3d_pattern.png")

plt.show()

# =============================================================================
# SUMMARY
# =============================================================================

print(f"\n{'='*70}")
print(f"SUMMARY")
print(f"{'='*70}")
print(f"\nPatch Antenna @ {f0/1e9:.0f} GHz on RO4350B:")
print(f"  Dimensions: {W*1e3:.3f} x {L*1e3:.3f} mm")
print(f"  Beamwidth:  E={bw_e:.1f} deg, H={bw_h:.1f} deg")
print(f"  Gain:       ~{G_element_dBi:.1f} dBi")
print(f"\nTX Array ({N_tx} elements, 2d spacing):")
print(f"  Beamwidth:  {bw_tx:.1f} deg")
print(f"  Gain:       ~{G_tx_dBi:.1f} dBi")
print(f"\nRX Array ({N_rx} elements, d spacing):")
print(f"  Beamwidth:  {bw_rx:.1f} deg")
print(f"  Gain:       ~{G_rx_dBi:.1f} dBi")
print(f"\nMIMO Virtual Array ({N_virtual} elements):")
print(f"  Beamwidth:  {bw_virtual:.1f} deg")
print(f"  Gain:       ~{G_virtual_dBi:.1f} dBi")
print(f"  Aperture:   {(N_virtual-1)*d_element*1e3:.2f} mm")
