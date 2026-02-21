"""
Radar Configuration Module

Defines the RadarConfig dataclass holding all radar system parameters for the
77 GHz FMCW SAR radar based on the TI AWR2243 cascade evaluation module.

The default configuration targets:
    - Center frequency: 78.5 GHz (77-80 GHz band)
    - Bandwidth: 5 GHz (range resolution ~3 cm)
    - Chirp duration: 60 us
    - ADC sample rate: 10 MSPS
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

import numpy as np


# Speed of light in m/s
C = 299_792_458.0


@dataclass
class RadarConfig:
    """Configuration parameters for the 77 GHz FMCW SAR radar.

    This dataclass holds all the fundamental radar parameters required by the
    SAR processing pipeline. Derived quantities (wavelength, range resolution,
    etc.) are exposed as properties computed on the fly from the base parameters.

    Attributes:
        fc: Center frequency in Hz. Default 78.5 GHz (middle of 77-80 GHz band).
        bw: Chirp sweep bandwidth in Hz. Default 5 GHz.
        t_chirp: Active chirp sweep duration in seconds. Default 60 us.
        t_idle: Idle time between consecutive chirps in seconds. Default 5 us.
        fs: ADC sampling rate in Hz. Default 10 MHz.
        num_samples: Number of ADC samples acquired per chirp. Default 512.
        num_chirps: Number of chirps per radar frame. Default 128.
        num_tx: Number of physical transmit antennas. Default 3.
        num_rx: Number of physical receive antennas. Default 4.
        num_virtual: Number of MIMO virtual antennas (num_tx * num_rx). Default 12.
        antenna_spacing_m: Physical spacing between adjacent virtual array
            elements in meters. Default is lambda/2 at 78.5 GHz.
    """

    fc: float = 78.5e9
    bw: float = 5.0e9
    t_chirp: float = 60.0e-6
    t_idle: float = 5.0e-6
    fs: float = 10.0e6
    num_samples: int = 512
    num_chirps: int = 128
    num_tx: int = 3
    num_rx: int = 4
    num_virtual: int = 12
    antenna_spacing_m: Optional[float] = None

    def __post_init__(self) -> None:
        """Set derived defaults after initialization."""
        if self.antenna_spacing_m is None:
            self.antenna_spacing_m = self.wavelength / 2.0

    # ------------------------------------------------------------------ #
    #  Derived properties
    # ------------------------------------------------------------------ #

    @property
    def wavelength(self) -> float:
        """Wavelength at center frequency in meters.

        lambda = c / fc
        At 78.5 GHz this is approximately 3.82 mm.
        """
        return C / self.fc

    @property
    def range_resolution(self) -> float:
        """Range resolution in meters.

        delta_R = c / (2 * BW)
        With 5 GHz bandwidth this yields ~3 cm.
        """
        return C / (2.0 * self.bw)

    @property
    def max_range(self) -> float:
        """Maximum unambiguous range in meters.

        R_max = fs * c / (2 * slope)
        where slope = BW / T_chirp.
        """
        slope = self.bw / self.t_chirp
        return self.fs * C / (2.0 * slope)

    @property
    def chirp_slope(self) -> float:
        """Chirp frequency slope in Hz/s.

        S = BW / T_chirp
        """
        return self.bw / self.t_chirp

    @property
    def pri(self) -> float:
        """Pulse repetition interval in seconds (chirp + idle)."""
        return self.t_chirp + self.t_idle

    @property
    def prf(self) -> float:
        """Pulse repetition frequency in Hz."""
        return 1.0 / self.pri

    @property
    def velocity_resolution(self) -> float:
        """Velocity resolution in m/s.

        delta_v = lambda / (2 * N_chirps * PRI)
        """
        return self.wavelength / (2.0 * self.num_chirps * self.pri)

    @property
    def max_velocity(self) -> float:
        """Maximum unambiguous velocity in m/s.

        v_max = lambda / (4 * PRI)
        """
        return self.wavelength / (4.0 * self.pri)

    @property
    def range_axis(self) -> np.ndarray:
        """Compute the range axis vector in meters.

        Returns an array of shape (num_samples,) representing the range bin
        centers from 0 to max_range.
        """
        return np.linspace(0, self.max_range, self.num_samples)

    @property
    def fast_time_axis(self) -> np.ndarray:
        """ADC sampling time axis (fast time) in seconds."""
        return np.arange(self.num_samples) / self.fs

    # ------------------------------------------------------------------ #
    #  Serialisation
    # ------------------------------------------------------------------ #

    def to_dict(self) -> dict:
        """Serialize configuration to a plain dictionary."""
        return asdict(self)

    def to_json(self, path: str | Path) -> None:
        """Save configuration to a JSON file.

        Args:
            path: Output file path.
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as fp:
            json.dump(self.to_dict(), fp, indent=4)

    @classmethod
    def from_file(cls, path: str | Path) -> "RadarConfig":
        """Load radar configuration from a JSON file.

        Args:
            path: Path to the JSON configuration file.

        Returns:
            A RadarConfig instance populated from the file.
        """
        path = Path(path)
        with open(path, "r") as fp:
            data = json.load(fp)
        return cls(**data)

    @classmethod
    def default(cls) -> "RadarConfig":
        """Return the default configuration for the AWR2243 77 GHz radar.

        This matches the default chirp configuration programmed into the
        AWR2243 cascade radar by the FPGA controller firmware.

        Returns:
            A RadarConfig with default AWR2243 parameters.
        """
        return cls()

    # ------------------------------------------------------------------ #
    #  Display
    # ------------------------------------------------------------------ #

    def summary(self) -> str:
        """Return a human-readable summary of the radar configuration."""
        lines = [
            "=== Radar Configuration ===",
            f"  Center frequency : {self.fc / 1e9:.2f} GHz",
            f"  Bandwidth        : {self.bw / 1e9:.2f} GHz",
            f"  Chirp duration   : {self.t_chirp * 1e6:.1f} us",
            f"  Idle time        : {self.t_idle * 1e6:.1f} us",
            f"  ADC sample rate  : {self.fs / 1e6:.1f} MSPS",
            f"  Samples / chirp  : {self.num_samples}",
            f"  Chirps / frame   : {self.num_chirps}",
            f"  TX / RX / Virtual: {self.num_tx} / {self.num_rx} / {self.num_virtual}",
            f"  --- Derived ---",
            f"  Wavelength       : {self.wavelength * 1e3:.3f} mm",
            f"  Range resolution : {self.range_resolution * 100:.2f} cm",
            f"  Max range        : {self.max_range:.2f} m",
            f"  Chirp slope      : {self.chirp_slope / 1e12:.2f} THz/s",
            f"  PRF              : {self.prf:.1f} Hz",
            f"  Vel. resolution  : {self.velocity_resolution:.4f} m/s",
            f"  Max velocity     : {self.max_velocity:.2f} m/s",
        ]
        return "\n".join(lines)

    def __repr__(self) -> str:
        return (
            f"RadarConfig(fc={self.fc / 1e9:.1f}GHz, bw={self.bw / 1e9:.1f}GHz, "
            f"samples={self.num_samples}, chirps={self.num_chirps})"
        )
