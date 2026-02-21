#!/usr/bin/env python3
"""
77 GHz FMCW SAR Radar - Link Budget Calculator
================================================

Calculates and plots:
- Received power vs range for various target RCS values
- SNR vs range with and without SAR processing gain
- Maximum detection range for given SNR threshold
- Complete link budget table

Based on AWR2243 radar transceiver specifications.

Author: SAR Radar Project
Date: 2026-02-21
"""

import numpy as np
import matplotlib.pyplot as plt
import os


# =============================================================================
# SYSTEM PARAMETERS
# =============================================================================

# Physical constants
c = 3e8                    # Speed of light [m/s]
k_boltz = 1.38e-23        # Boltzmann constant [J/K]
T0 = 290                  # Standard temperature [K]

# Radar frequency
fc = 77e9                  # Center frequency [Hz]
lam = c / fc               # Wavelength [m]

# Transmitter (AWR2243)
Pt_dBm = 12.0             # Transmit power [dBm]
Pt_W = 10 ** ((Pt_dBm - 30) / 10)  # Transmit power [W]

# Antenna gains
Gt_dBi = 10.0             # TX antenna gain [dBi] (3-element patch array)
Gr_dBi = 11.0             # RX antenna gain [dBi] (4-element patch array)
Gt = 10 ** (Gt_dBi / 10)  # TX gain linear
Gr = 10 ** (Gr_dBi / 10)  # RX gain linear

# Receiver (AWR2243)
NF_dB = 12.0              # Noise figure [dB]
NF = 10 ** (NF_dB / 10)   # Noise figure linear
B_IF = 10e6                # IF bandwidth [Hz] (ADC sampling rate)

# SAR processing parameters
N_pulses = 300             # Number of pulses in synthetic aperture (300 positions)
BW_chirp = 5e9             # Chirp bandwidth [Hz]
T_chirp = 60e-6            # Chirp duration [s]
N_samples = 512            # ADC samples per chirp

# Target parameters
RCS_values = [0.01, 0.05, 0.1, 0.5, 1.0]  # RCS values [m^2]
RCS_labels = ["0.01 m^2 (small)", "0.05 m^2", "0.1 m^2 (person)",
              "0.5 m^2", "1.0 m^2 (large)"]

# Range axis
R = np.linspace(0.3, 5.0, 500)  # Range [m]

# Losses
L_atm_dB_per_m = 0.02     # Atmospheric loss at 77 GHz [dB/m] (approx.)
L_sys_dB = 3.0             # System losses (cables, mismatch, etc.) [dB]
L_sys = 10 ** (L_sys_dB / 10)

# SNR thresholds
SNR_detect_dB = 10.0       # Detection threshold [dB]
SNR_image_dB = 15.0        # Imaging threshold (better quality) [dB]

print("=" * 70)
print("77 GHz FMCW SAR RADAR - LINK BUDGET ANALYSIS")
print("=" * 70)

print(f"\nSystem Parameters:")
print(f"  Frequency:        {fc/1e9:.0f} GHz")
print(f"  Wavelength:       {lam*1e3:.2f} mm")
print(f"  TX Power:         {Pt_dBm:.1f} dBm ({Pt_W*1e3:.2f} mW)")
print(f"  TX Antenna Gain:  {Gt_dBi:.1f} dBi")
print(f"  RX Antenna Gain:  {Gr_dBi:.1f} dBi")
print(f"  Noise Figure:     {NF_dB:.1f} dB")
print(f"  IF Bandwidth:     {B_IF/1e6:.0f} MHz")
print(f"  System Losses:    {L_sys_dB:.1f} dB")
print(f"  Atm. Loss:        {L_atm_dB_per_m:.2f} dB/m")


# =============================================================================
# RADAR EQUATION CALCULATIONS
# =============================================================================

# Noise power
Pn = k_boltz * T0 * B_IF * NF  # [W]
Pn_dBm = 10 * np.log10(Pn) + 30  # [dBm]

print(f"\nNoise Floor:")
print(f"  Thermal noise (kTB):  {10*np.log10(k_boltz*T0*B_IF)+30:.1f} dBm")
print(f"  With NF ({NF_dB:.0f} dB):    {Pn_dBm:.1f} dBm")

# Range compression gain
# After range FFT, noise bandwidth reduces from B_IF to 1/T_chirp
# But signal is coherently integrated across N_samples
range_compression_gain = N_samples  # Coherent integration in fast-time FFT
range_compression_gain_dB = 10 * np.log10(range_compression_gain)

# SAR processing gain (coherent integration of N_pulses)
# Amplitude gain = sqrt(N_pulses), power gain = N_pulses
sar_gain = N_pulses
sar_gain_dB = 10 * np.log10(sar_gain)

print(f"\nProcessing Gains:")
print(f"  Range compression:  {range_compression_gain_dB:.1f} dB ({N_samples} samples)")
print(f"  SAR integration:    {sar_gain_dB:.1f} dB ({N_pulses} pulses)")
print(f"  Total processing:   {range_compression_gain_dB + sar_gain_dB:.1f} dB")


def radar_equation_snr(R, sigma, Pt_W, Gt, Gr, lam, Pn, L_sys, L_atm_dB_per_m):
    """
    Calculate single-pulse SNR using the radar equation.

    Pr = Pt * Gt * Gr * lam^2 * sigma / ((4*pi)^3 * R^4 * L_sys * L_atm)
    SNR = Pr / Pn

    Parameters
    ----------
    R : ndarray - Range [m]
    sigma : float - Radar cross section [m^2]
    Others : float - System parameters

    Returns
    -------
    SNR_dB : ndarray - SNR in dB
    Pr_dBm : ndarray - Received power in dBm
    """
    # Atmospheric loss (two-way)
    L_atm = 10 ** (L_atm_dB_per_m * 2 * R / 10)

    # Received power [W]
    Pr = (Pt_W * Gt * Gr * lam ** 2 * sigma) / (
        (4 * np.pi) ** 3 * R ** 4 * L_sys * L_atm
    )

    Pr_dBm = 10 * np.log10(Pr + 1e-30) + 30

    # SNR
    SNR = Pr / Pn
    SNR_dB = 10 * np.log10(SNR + 1e-30)

    return SNR_dB, Pr_dBm


def find_max_range(R, SNR_dB, threshold_dB):
    """Find maximum range where SNR exceeds threshold."""
    above = SNR_dB >= threshold_dB
    if not np.any(above):
        return 0.0
    # Find last index where SNR is above threshold
    last_idx = np.where(above)[0][-1]
    return R[last_idx]


# Calculate for each RCS value
print(f"\n{'='*70}")
print(f"RECEIVED POWER AND SNR ANALYSIS")
print(f"{'='*70}")

results = {}
for sigma, label in zip(RCS_values, RCS_labels):
    SNR_single, Pr = radar_equation_snr(R, sigma, Pt_W, Gt, Gr, lam, Pn, L_sys, L_atm_dB_per_m)

    # Add range compression gain
    SNR_range_compressed = SNR_single + range_compression_gain_dB

    # Add SAR processing gain
    SNR_sar = SNR_range_compressed + sar_gain_dB

    # Max detection ranges
    R_max_single = find_max_range(R, SNR_single, SNR_detect_dB)
    R_max_rc = find_max_range(R, SNR_range_compressed, SNR_detect_dB)
    R_max_sar = find_max_range(R, SNR_sar, SNR_detect_dB)
    R_max_sar_img = find_max_range(R, SNR_sar, SNR_image_dB)

    results[sigma] = {
        "label": label,
        "Pr": Pr,
        "SNR_single": SNR_single,
        "SNR_rc": SNR_range_compressed,
        "SNR_sar": SNR_sar,
        "R_max_single": R_max_single,
        "R_max_rc": R_max_rc,
        "R_max_sar": R_max_sar,
        "R_max_sar_img": R_max_sar_img,
    }

# Print summary table
print(f"\n  {'RCS':<18} {'R_max(SP)':<12} {'R_max(RC)':<12} "
      f"{'R_max(SAR)':<12} {'R_max(img)':<12}")
print(f"  {'[m^2]':<18} {'SNR>10dB':<12} {'SNR>10dB':<12} "
      f"{'SNR>10dB':<12} {'SNR>15dB':<12}")
print(f"  {'-'*66}")

for sigma in RCS_values:
    r = results[sigma]
    print(f"  {r['label']:<18} {r['R_max_single']:<12.2f} {r['R_max_rc']:<12.2f} "
          f"{r['R_max_sar']:<12.2f} {r['R_max_sar_img']:<12.2f}")

print(f"\n  SP  = Single Pulse (no processing)")
print(f"  RC  = After Range Compression (+{range_compression_gain_dB:.1f} dB)")
print(f"  SAR = After SAR Integration (+{sar_gain_dB:.1f} dB additional)")

# Detailed link budget at reference range
R_ref = 2.0  # Reference range [m]
sigma_ref = 0.1  # Reference RCS [m^2]

L_atm_ref = L_atm_dB_per_m * 2 * R_ref
free_space_loss = 20 * np.log10(4 * np.pi * R_ref / lam)

print(f"\n{'='*70}")
print(f"DETAILED LINK BUDGET (R={R_ref:.1f}m, RCS={sigma_ref} m^2)")
print(f"{'='*70}")
print(f"\n  {'Parameter':<35} {'Value':<15} {'Unit'}")
print(f"  {'-'*60}")
print(f"  {'TX Power':<35} {Pt_dBm:<15.1f} {'dBm'}")
print(f"  {'TX Antenna Gain':<35} {Gt_dBi:<15.1f} {'dBi'}")
print(f"  {'EIRP':<35} {Pt_dBm+Gt_dBi:<15.1f} {'dBm'}")
print(f"  {'Free-Space Path Loss (one-way)':<35} {-free_space_loss:<15.1f} {'dB'}")
print(f"  {'Atmospheric Loss (two-way)':<35} {-L_atm_ref:<15.2f} {'dB'}")
print(f"  {'Target RCS (10log10)':<35} {10*np.log10(sigma_ref):<15.1f} {'dBsm'}")
print(f"  {'RX Antenna Gain':<35} {Gr_dBi:<15.1f} {'dBi'}")
print(f"  {'System Losses':<35} {-L_sys_dB:<15.1f} {'dB'}")
print(f"  {'Spreading Factor (4pi)^3':<35} {-10*np.log10((4*np.pi)**3):<15.1f} {'dB'}")
print(f"  {'R^4 Factor':<35} {-40*np.log10(R_ref):<15.1f} {'dB'}")

# Calculate Pr at reference
SNR_ref, Pr_ref = radar_equation_snr(
    np.array([R_ref]), sigma_ref, Pt_W, Gt, Gr, lam, Pn, L_sys, L_atm_dB_per_m
)
print(f"  {'':<35} {'-------'}")
print(f"  {'Received Power':<35} {Pr_ref[0]:<15.1f} {'dBm'}")
print(f"  {'Noise Floor':<35} {Pn_dBm:<15.1f} {'dBm'}")
print(f"  {'Single-Pulse SNR':<35} {SNR_ref[0]:<15.1f} {'dB'}")
print(f"  {'+ Range Compression Gain':<35} {range_compression_gain_dB:<15.1f} {'dB'}")
print(f"  {'+ SAR Processing Gain':<35} {sar_gain_dB:<15.1f} {'dB'}")
print(f"  {'Total SNR (after SAR)':<35} {SNR_ref[0]+range_compression_gain_dB+sar_gain_dB:<15.1f} {'dB'}")


# =============================================================================
# PLOTTING
# =============================================================================

output_dir = os.path.dirname(os.path.abspath(__file__))

# ---------- Figure 1: Received Power vs Range ----------
fig1, ax1 = plt.subplots(figsize=(10, 6))
fig1.suptitle("Received Power vs Range", fontsize=14, fontweight="bold")

colors = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4", "#9467bd"]

for (sigma, data), color in zip(results.items(), colors):
    ax1.plot(R, data["Pr"], color=color, linewidth=1.5, label=data["label"])

ax1.axhline(y=Pn_dBm, color="black", linestyle="--", alpha=0.7, label=f"Noise floor ({Pn_dBm:.0f} dBm)")
ax1.set_xlabel("Range [m]")
ax1.set_ylabel("Received Power [dBm]")
ax1.set_xlim(R[0], R[-1])
ax1.grid(True, alpha=0.3)
ax1.legend(fontsize=9)

fig1.tight_layout()
fig1.savefig(os.path.join(output_dir, "link_budget_rx_power.png"), dpi=150, bbox_inches="tight")
print(f"\nSaved: link_budget_rx_power.png")

# ---------- Figure 2: SNR vs Range (single-pulse) ----------
fig2, axes2 = plt.subplots(1, 3, figsize=(18, 6))
fig2.suptitle("SNR vs Range for Different Processing Stages", fontsize=14, fontweight="bold")

titles = ["Single Pulse SNR", f"After Range Compression (+{range_compression_gain_dB:.0f}dB)",
          f"After SAR Processing (+{sar_gain_dB:.0f}dB)"]
keys = ["SNR_single", "SNR_rc", "SNR_sar"]

for ax, title, key in zip(axes2, titles, keys):
    for (sigma, data), color in zip(results.items(), colors):
        ax.plot(R, data[key], color=color, linewidth=1.5, label=data["label"])

    ax.axhline(y=SNR_detect_dB, color="black", linestyle="--", alpha=0.7,
               label=f"Detection threshold ({SNR_detect_dB:.0f} dB)")
    ax.axhline(y=SNR_image_dB, color="gray", linestyle=":", alpha=0.5,
               label=f"Imaging threshold ({SNR_image_dB:.0f} dB)")
    ax.set_xlabel("Range [m]")
    ax.set_ylabel("SNR [dB]")
    ax.set_title(title)
    ax.set_xlim(R[0], R[-1])
    ax.set_ylim(-20, 80)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="upper right")

fig2.tight_layout()
fig2.savefig(os.path.join(output_dir, "link_budget_snr.png"), dpi=150, bbox_inches="tight")
print(f"Saved: link_budget_snr.png")

# ---------- Figure 3: Maximum Detection Range Bar Chart ----------
fig3, ax3 = plt.subplots(figsize=(12, 6))
fig3.suptitle("Maximum Detection Range (SNR > 10 dB)", fontsize=14, fontweight="bold")

x_pos = np.arange(len(RCS_values))
bar_width = 0.25

bars1 = ax3.bar(x_pos - bar_width, [results[s]["R_max_single"] for s in RCS_values],
                bar_width, label="Single Pulse", color="#d62728", alpha=0.8)
bars2 = ax3.bar(x_pos, [results[s]["R_max_rc"] for s in RCS_values],
                bar_width, label="Range Compressed", color="#2ca02c", alpha=0.8)
bars3 = ax3.bar(x_pos + bar_width, [results[s]["R_max_sar"] for s in RCS_values],
                bar_width, label="SAR Processed", color="#1f77b4", alpha=0.8)

ax3.set_xlabel("Target RCS")
ax3.set_ylabel("Maximum Range [m]")
ax3.set_xticks(x_pos)
ax3.set_xticklabels([f"{s} m^2" for s in RCS_values])
ax3.legend()
ax3.grid(True, alpha=0.3, axis="y")

# Add value labels on bars
for bars in [bars1, bars2, bars3]:
    for bar in bars:
        height = bar.get_height()
        if height > 0:
            ax3.annotate(
                f"{height:.1f}m",
                xy=(bar.get_x() + bar.get_width() / 2, height),
                xytext=(0, 3), textcoords="offset points",
                ha="center", va="bottom", fontsize=7
            )

fig3.tight_layout()
fig3.savefig(os.path.join(output_dir, "link_budget_max_range.png"), dpi=150, bbox_inches="tight")
print(f"Saved: link_budget_max_range.png")

# ---------- Figure 4: SNR Components Waterfall ----------
fig4, ax4 = plt.subplots(figsize=(12, 7))
fig4.suptitle(f"Link Budget Waterfall (R={R_ref}m, RCS={sigma_ref} m^2)",
              fontsize=14, fontweight="bold")

# Build waterfall data
waterfall_items = [
    ("TX Power", Pt_dBm),
    ("TX Gain", Gt_dBi),
    ("Path Loss (2-way)", -2 * free_space_loss),
    ("Atm. Loss (2-way)", -L_atm_ref),
    ("Target RCS", 10 * np.log10(sigma_ref)),
    ("RX Gain", Gr_dBi),
    ("Spreading (4pi)^3", -10 * np.log10((4 * np.pi) ** 3)),
    ("R^4 Loss", -40 * np.log10(R_ref)),
    ("System Loss", -L_sys_dB),
]

labels = [item[0] for item in waterfall_items]
values = [item[1] for item in waterfall_items]

# Calculate cumulative
cumulative = np.cumsum(values)
starts = np.concatenate([[0], cumulative[:-1]])

# Color: green for gains, red for losses
bar_colors = ["green" if v > 0 else "red" for v in values]

# Plot
bars = ax4.barh(range(len(labels)), values, left=starts, color=bar_colors, alpha=0.7, edgecolor="black")

# Add value labels
for i, (start, val) in enumerate(zip(starts, values)):
    end = start + val
    x_text = end + 1 if val > 0 else end - 1
    ha = "left" if val > 0 else "right"
    ax4.text(x_text, i, f"{val:+.1f} dB", va="center", ha=ha, fontsize=9, fontweight="bold")

# Final received power line
ax4.axvline(x=cumulative[-1], color="blue", linestyle="-", linewidth=2, alpha=0.7)
ax4.text(cumulative[-1], len(labels) - 0.5, f"Pr = {cumulative[-1]:.1f}", fontsize=10,
         color="blue", fontweight="bold")

ax4.set_yticks(range(len(labels)))
ax4.set_yticklabels(labels)
ax4.set_xlabel("Cumulative [dB]")
ax4.invert_yaxis()
ax4.grid(True, alpha=0.3, axis="x")

fig4.tight_layout()
fig4.savefig(os.path.join(output_dir, "link_budget_waterfall.png"), dpi=150, bbox_inches="tight")
print(f"Saved: link_budget_waterfall.png")

plt.show()

# =============================================================================
# FINAL SUMMARY
# =============================================================================

print(f"\n{'='*70}")
print(f"LINK BUDGET SUMMARY")
print(f"{'='*70}")
print(f"\n  Operating frequency:       {fc/1e9:.0f} GHz")
print(f"  TX power:                  {Pt_dBm:.1f} dBm")
print(f"  TX/RX antenna gain:        {Gt_dBi:.1f}/{Gr_dBi:.1f} dBi")
print(f"  Noise figure:              {NF_dB:.1f} dB")
print(f"  IF bandwidth:              {B_IF/1e6:.0f} MHz")
print(f"  Noise floor:               {Pn_dBm:.1f} dBm")
print(f"  Range compression gain:    {range_compression_gain_dB:.1f} dB")
print(f"  SAR processing gain:       {sar_gain_dB:.1f} dB")
print(f"  Total processing gain:     {range_compression_gain_dB+sar_gain_dB:.1f} dB")
print(f"\n  For RCS = 0.1 m^2 (typical person):")
print(f"    Max range (single pulse):    {results[0.1]['R_max_single']:.2f} m")
print(f"    Max range (range comp.):     {results[0.1]['R_max_rc']:.2f} m")
print(f"    Max range (SAR processed):   {results[0.1]['R_max_sar']:.2f} m")
print(f"    Max range (imaging quality): {results[0.1]['R_max_sar_img']:.2f} m")
print(f"\n  Conclusion: SAR processing extends detection range significantly.")
print(f"  For handheld operation (0.5-3m range), the system has adequate margin")
print(f"  for targets with RCS >= 0.01 m^2 after SAR processing.")
