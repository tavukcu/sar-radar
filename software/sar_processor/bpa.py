"""
Back-Projection Algorithm (BPA) for SAR Image Formation

This module implements the time-domain Back-Projection Algorithm, which is the
gold-standard SAR image formation technique for non-linear flight paths (such
as those produced by handheld scanning). Unlike frequency-domain algorithms
(omega-K, Range-Doppler), BPA handles arbitrary motion without approximations.

## FMCW SAR Signal Model

For an FMCW (Frequency Modulated Continuous Wave) radar, the transmitted chirp
signal at slow-time index n is:

    s_tx(t) = exp(j * 2*pi * (fc*t + S/2 * t^2))

where:
    - fc is the carrier (center) frequency
    - S = BW / T_chirp is the chirp slope (Hz/s)
    - t is the fast-time within the chirp [0, T_chirp]

After mixing with the received echo from a point target at range R, the
intermediate frequency (IF) signal is:

    s_IF(t) = exp(j * 4*pi*fc*R/c) * exp(j * 4*pi*S*R/(c) * t)

The first exponential is the residual video phase (range-dependent constant
phase), and the second produces a beat frequency f_b = 2*S*R/c.

## Range Compression

Range compression is performed by taking the FFT along the fast-time axis.
The beat frequency maps directly to range:

    R = f_b * c / (2*S)

After FFT, we obtain the range-compressed signal s_rc(n, R) where n is the
slow-time (chirp/pulse) index.

## Back-Projection

The SAR image is formed by coherent summation over all slow-time positions:

    I(x, y) = sum_n { s_rc(n, R_n(x,y)) * exp(j * 2*pi*fc * 2*R_n(x,y) / c) }

where:
    R_n(x, y) = || r_antenna(n) - r_pixel(x, y) ||

is the range from the antenna position at pulse n to the pixel at (x, y).
The exponential provides the matched-filter phase compensation for the
residual video phase.

The interpolation of s_rc at non-integer range bins is handled by sinc
interpolation (ideal) or linear/cubic interpolation (practical).
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import windows

from sar_processor.radar_config import RadarConfig, C


class BackProjectionAlgorithm:
    """Time-domain Back-Projection SAR image formation.

    This class performs range compression on raw IF data and then applies the
    back-projection algorithm to form a 2D or 3D SAR image over a specified
    pixel grid.

    Usage:
        config = RadarConfig.default()
        bpa = BackProjectionAlgorithm(config)

        # Range compression
        rc_data = bpa.range_compression(raw_if_data)

        # Setup output image grid
        grid_x, grid_y = bpa.setup_grid(
            x_extent=(-0.5, 0.5), y_extent=(0.1, 1.0), pixel_size=0.005
        )

        # Back-project
        image = bpa.backproject(rc_data, antenna_positions, grid_x, grid_y)
    """

    def __init__(
        self,
        config: RadarConfig,
        window: str = "hann",
        interp_kind: str = "linear",
    ) -> None:
        """Initialize the BPA processor.

        Args:
            config: Radar configuration parameters.
            window: Window function to apply before range FFT. One of
                'hann', 'hamming', 'blackman', 'kaiser', or 'none'.
            interp_kind: Interpolation method for range profile resampling.
                One of 'linear', 'cubic', 'nearest'.
        """
        self.config = config
        self.window_type = window
        self.interp_kind = interp_kind

        # Precompute range axis for range-compressed data
        self._range_axis: Optional[np.ndarray] = None
        self._window_func: Optional[np.ndarray] = None

    def _get_window(self, n: int) -> np.ndarray:
        """Get the window function for range FFT.

        Args:
            n: Window length (number of samples).

        Returns:
            Window coefficients array of length n.
        """
        if self.window_type == "none" or self.window_type is None:
            return np.ones(n)
        elif self.window_type == "hann":
            return windows.hann(n)
        elif self.window_type == "hamming":
            return windows.hamming(n)
        elif self.window_type == "blackman":
            return windows.blackman(n)
        elif self.window_type == "kaiser":
            return windows.kaiser(n, beta=6.0)
        else:
            raise ValueError(f"Unknown window type: {self.window_type}")

    def range_compression(
        self,
        raw_data: np.ndarray,
        subtract_mean: bool = True,
        zero_pad_factor: int = 1,
    ) -> np.ndarray:
        """Perform range compression via FFT on raw IF data.

        Range compression converts the beat-frequency IF signal into the range
        domain. This is done by taking the FFT along the fast-time (sample)
        axis. An optional window is applied to reduce range sidelobes.

        The beat frequency to range mapping is:
            R = f_b * c / (2 * S)  where S = BW / T_chirp

        Args:
            raw_data: Raw IF data with shape (..., num_samples) where the last
                axis is the fast-time (ADC sample) dimension. Typical shapes:
                - (num_chirps, num_samples) for single channel
                - (num_chirps, num_rx, num_samples) for multi-channel
            subtract_mean: If True, subtract the mean along each chirp to
                remove DC offset / direct coupling leakage.
            zero_pad_factor: Zero-padding factor for FFT (1 = no padding,
                2 = double the FFT size for finer range sampling).

        Returns:
            Range-compressed complex data with same leading dimensions and
            last axis of length num_samples * zero_pad_factor.
        """
        num_samples = raw_data.shape[-1]
        fft_size = num_samples * zero_pad_factor

        # Get / cache window function
        if self._window_func is None or len(self._window_func) != num_samples:
            self._window_func = self._get_window(num_samples)

        # Subtract mean (DC removal) to suppress zero-range leakage
        if subtract_mean:
            data = raw_data - np.mean(raw_data, axis=-1, keepdims=True)
        else:
            data = raw_data.copy()

        # Apply window along fast-time axis
        data = data * self._window_func

        # Range FFT
        rc_data = np.fft.fft(data, n=fft_size, axis=-1)

        # Keep only positive frequencies (real targets at positive ranges)
        rc_data = rc_data[..., : fft_size // 2]

        # Compute range axis
        slope = self.config.chirp_slope
        freq_axis = np.fft.fftfreq(fft_size, d=1.0 / self.config.fs)[:fft_size // 2]
        self._range_axis = freq_axis * C / (2.0 * slope)

        return rc_data

    @property
    def range_axis(self) -> np.ndarray:
        """Range axis in meters for the range-compressed data.

        Available after calling range_compression().
        """
        if self._range_axis is None:
            # Compute default range axis
            fft_size = self.config.num_samples
            slope = self.config.chirp_slope
            freq_axis = np.fft.fftfreq(fft_size, d=1.0 / self.config.fs)[:fft_size // 2]
            self._range_axis = freq_axis * C / (2.0 * slope)
        return self._range_axis

    def setup_grid(
        self,
        x_extent: Tuple[float, float] = (-0.5, 0.5),
        y_extent: Tuple[float, float] = (0.1, 1.0),
        z_extent: Optional[Tuple[float, float]] = None,
        pixel_size: float = 0.005,
    ) -> Tuple[np.ndarray, ...]:
        """Create the output image pixel grid.

        Sets up a rectilinear grid of pixel positions for the SAR image. The
        coordinate system is:
            - x: cross-range (horizontal, perpendicular to look direction)
            - y: range (along look direction, away from radar)
            - z: elevation (optional, for 3D imaging)

        Args:
            x_extent: (x_min, x_max) in meters for the cross-range dimension.
            y_extent: (y_min, y_max) in meters for the range dimension.
            z_extent: Optional (z_min, z_max) for 3D imaging. If None, 2D grid.
            pixel_size: Grid spacing in meters. Determines output resolution.

        Returns:
            For 2D: tuple of (grid_x, grid_y) 2D meshgrid arrays.
            For 3D: tuple of (grid_x, grid_y, grid_z) 3D meshgrid arrays.
        """
        x_axis = np.arange(x_extent[0], x_extent[1] + pixel_size, pixel_size)
        y_axis = np.arange(y_extent[0], y_extent[1] + pixel_size, pixel_size)

        if z_extent is not None:
            z_axis = np.arange(z_extent[0], z_extent[1] + pixel_size, pixel_size)
            grid_x, grid_y, grid_z = np.meshgrid(x_axis, y_axis, z_axis, indexing="xy")
            return grid_x, grid_y, grid_z

        grid_x, grid_y = np.meshgrid(x_axis, y_axis, indexing="xy")
        return grid_x, grid_y

    def backproject(
        self,
        rc_data: np.ndarray,
        antenna_positions: np.ndarray,
        grid_x: np.ndarray,
        grid_y: np.ndarray,
        grid_z: Optional[np.ndarray] = None,
        verbose: bool = False,
    ) -> np.ndarray:
        """Perform back-projection to form the SAR image.

        For each pixel in the output grid, compute the round-trip distance to
        each antenna position, look up the corresponding range-compressed value
        (with interpolation), apply phase compensation, and coherently sum
        across all pulses.

        The back-projection formula:
            I(x,y) = sum_n { s_rc(n, R_n) * exp(j * 4*pi*fc*R_n / c) }

        where R_n(x,y) = || pos_n - (x,y,z) || is the distance from antenna
        position at pulse n to the pixel.

        Note: The range-compressed data already contains the phase
        exp(j*4*pi*S*R/(c)*t_beat), so we apply the residual video phase
        (RVP) correction exp(j*4*pi*fc*R/c) to achieve coherent focusing.

        Args:
            rc_data: Range-compressed data of shape (num_pulses, num_range_bins)
                or (num_pulses, num_rx, num_range_bins). If multi-channel, the
                channels are summed coherently (assumes pre-calibration).
            antenna_positions: Array of shape (num_pulses, 3) giving the [x, y, z]
                position of the antenna phase center for each pulse.
            grid_x: 2D or 3D array of pixel x-coordinates (meters).
            grid_y: 2D or 3D array of pixel y-coordinates (meters).
            grid_z: Optional 3D array of pixel z-coordinates. If None, assume z=0.
            verbose: If True, print progress during back-projection.

        Returns:
            Complex SAR image array with the same shape as grid_x / grid_y.
        """
        num_pulses = rc_data.shape[0]

        # Handle multi-channel data: average across RX channels
        if rc_data.ndim == 3:
            # (num_pulses, num_rx, num_range_bins) -> (num_pulses, num_range_bins)
            rc_data = np.mean(rc_data, axis=1)

        num_range_bins = rc_data.shape[-1]
        range_axis = self.range_axis

        # Ensure range axis matches data
        if len(range_axis) != num_range_bins:
            slope = self.config.chirp_slope
            freq_axis = np.fft.fftfreq(num_range_bins * 2, d=1.0 / self.config.fs)[
                :num_range_bins
            ]
            range_axis = freq_axis * C / (2.0 * slope)

        # Pixel positions for z dimension
        if grid_z is None:
            grid_z_flat = np.zeros(grid_x.size)
        else:
            grid_z_flat = grid_z.ravel()

        output_shape = grid_x.shape
        grid_x_flat = grid_x.ravel()
        grid_y_flat = grid_y.ravel()
        num_pixels = grid_x_flat.size

        # Carrier wavenumber for phase compensation
        k_c = 4.0 * np.pi * self.config.fc / C

        # Initialize output image
        image = np.zeros(num_pixels, dtype=np.complex128)

        # Build interpolators for each pulse (vectorized over pixels)
        for n in range(num_pulses):
            if verbose and n % 100 == 0:
                print(f"  Back-projecting pulse {n}/{num_pulses}")

            # Antenna position for this pulse
            ax, ay, az = antenna_positions[n, 0], antenna_positions[n, 1], antenna_positions[n, 2]

            # Compute distance from antenna to every pixel
            dx = grid_x_flat - ax
            dy = grid_y_flat - ay
            dz = grid_z_flat - az
            distances = np.sqrt(dx * dx + dy * dy + dz * dz)

            # Interpolate range-compressed data at computed distances
            # Use linear interpolation for speed (cubic for quality)
            rc_interp = np.interp(distances, range_axis, np.real(rc_data[n, :])) + \
                        1j * np.interp(distances, range_axis, np.imag(rc_data[n, :]))

            # Phase compensation (residual video phase)
            phase_comp = np.exp(1j * k_c * distances)

            # Coherent summation
            image += rc_interp * phase_comp

        return image.reshape(output_shape)

    def backproject_vectorized(
        self,
        rc_data: np.ndarray,
        antenna_positions: np.ndarray,
        grid_x: np.ndarray,
        grid_y: np.ndarray,
        grid_z: Optional[np.ndarray] = None,
        batch_size: int = 64,
    ) -> np.ndarray:
        """Vectorized back-projection with pulse batching for better performance.

        This version processes multiple pulses simultaneously by batching the
        distance computation and interpolation. It trades memory for speed
        and is significantly faster than the basic loop version.

        Args:
            rc_data: Range-compressed data, shape (num_pulses, num_range_bins).
            antenna_positions: Shape (num_pulses, 3).
            grid_x: Pixel x-coordinates.
            grid_y: Pixel y-coordinates.
            grid_z: Optional pixel z-coordinates.
            batch_size: Number of pulses to process simultaneously.

        Returns:
            Complex SAR image array.
        """
        num_pulses = rc_data.shape[0]

        if rc_data.ndim == 3:
            rc_data = np.mean(rc_data, axis=1)

        num_range_bins = rc_data.shape[-1]
        range_axis = self.range_axis
        if len(range_axis) != num_range_bins:
            slope = self.config.chirp_slope
            freq_axis = np.fft.fftfreq(num_range_bins * 2, d=1.0 / self.config.fs)[
                :num_range_bins
            ]
            range_axis = freq_axis * C / (2.0 * slope)

        output_shape = grid_x.shape
        grid_x_flat = grid_x.ravel()
        grid_y_flat = grid_y.ravel()
        num_pixels = grid_x_flat.size

        if grid_z is not None:
            grid_z_flat = grid_z.ravel()
        else:
            grid_z_flat = np.zeros(num_pixels)

        k_c = 4.0 * np.pi * self.config.fc / C
        image = np.zeros(num_pixels, dtype=np.complex128)

        # Pixel positions as (num_pixels, 3)
        pixel_pos = np.stack([grid_x_flat, grid_y_flat, grid_z_flat], axis=-1)

        # Range bin indices for fast interpolation
        range_step = range_axis[1] - range_axis[0] if len(range_axis) > 1 else 1.0
        range_start = range_axis[0]

        for batch_start in range(0, num_pulses, batch_size):
            batch_end = min(batch_start + batch_size, num_pulses)
            batch_positions = antenna_positions[batch_start:batch_end]  # (B, 3)
            batch_rc = rc_data[batch_start:batch_end]  # (B, num_range_bins)

            # Compute distances: (B, num_pixels)
            diff = pixel_pos[np.newaxis, :, :] - batch_positions[:, np.newaxis, :]  # (B, P, 3)
            distances = np.linalg.norm(diff, axis=-1)  # (B, P)

            # Convert distances to fractional range bin indices
            frac_indices = (distances - range_start) / range_step
            frac_indices = np.clip(frac_indices, 0, num_range_bins - 1.001)
            idx_low = frac_indices.astype(np.int32)
            idx_high = np.minimum(idx_low + 1, num_range_bins - 1)
            frac = frac_indices - idx_low

            # Linear interpolation across the batch
            for b in range(batch_end - batch_start):
                rc_low = batch_rc[b, idx_low[b]]
                rc_high = batch_rc[b, idx_high[b]]
                rc_interp = rc_low * (1.0 - frac[b]) + rc_high * frac[b]

                phase_comp = np.exp(1j * k_c * distances[b])
                image += rc_interp * phase_comp

        return image.reshape(output_shape)

    @staticmethod
    def to_db(image: np.ndarray, dynamic_range: float = 60.0) -> np.ndarray:
        """Convert complex SAR image to dB scale.

        Args:
            image: Complex SAR image.
            dynamic_range: Dynamic range to display in dB.

        Returns:
            Image in dB scale, clipped to [max - dynamic_range, max].
        """
        magnitude = np.abs(image)
        magnitude = np.where(magnitude > 0, magnitude, 1e-30)
        db = 20.0 * np.log10(magnitude)
        db_max = np.max(db)
        db = np.clip(db, db_max - dynamic_range, db_max)
        return db
