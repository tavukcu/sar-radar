"""
Phase Gradient Autofocus (PGA)

Implements the Phase Gradient Autofocus algorithm for correcting residual
phase errors in SAR imagery. PGA is a data-driven, non-parametric autofocus
technique that estimates and removes phase errors without requiring knowledge
of the radar platform motion.

PGA is particularly important for handheld SAR where IMU-based motion
compensation leaves residual phase errors due to:
    - IMU drift and noise
    - Imperfect trajectory estimation
    - Phase errors from hardware (oscillator jitter, cable flex)

## Algorithm Overview

PGA operates on the complex SAR image in the range-compressed, azimuth-
frequency domain and iterates through these steps:

1. **Center Shifting**: Circularly shift each range line so that the
   dominant scatterer is centered. This maximizes the correlation of the
   phase error across range lines.

2. **Windowing**: Apply a data-adaptive window (or fixed Hamming window)
   to select the strong scatterer region and suppress noise.

3. **Phase Gradient Estimation**: Estimate the derivative of the phase
   error using the Maximum Likelihood (ML) estimator:

       phi'(n) = Im{ sum_m [ s_m(n) * conj(s_m(n-1)) ] }
                / sum_m [ |s_m(n-1)|^2 ]

   where s_m(n) is the windowed signal at range line m, azimuth index n.

4. **Phase Integration**: Integrate the phase gradient to recover the
   absolute phase error:

       phi(n) = sum_{k=0}^{n} phi'(k)

   Remove the linear component (corresponds to bulk shift, not error).

5. **Phase Correction**: Apply the conjugate of the estimated phase error
   to the original data:

       s_corrected(n) = s(n) * exp(-j * phi(n))

6. **Iteration**: Repeat steps 1-5 until the phase error converges
   (typically 3-5 iterations).

References:
    - Wahl, D.E., et al., "Phase Gradient Autofocus - A Robust Tool for
      High Resolution SAR Phase Correction," IEEE Trans. AES, 1994.
    - Jakowatz, C.V., et al., "Spotlight-Mode Synthetic Aperture Radar:
      A Signal Processing Approach," Kluwer, 1996.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.signal import windows


class PhaseGradientAutofocus:
    """Phase Gradient Autofocus (PGA) for SAR image quality improvement.

    This class implements the iterative PGA algorithm to estimate and correct
    residual phase errors in SAR images. It operates on the complex image
    data and returns the corrected image.

    Usage:
        pga = PhaseGradientAutofocus(max_iter=5, convergence_threshold=0.01)
        corrected_image = pga.apply(complex_sar_image)
    """

    def __init__(
        self,
        max_iter: int = 5,
        convergence_threshold: float = 0.01,
        window_type: str = "hamming",
        window_fraction: float = 0.5,
    ) -> None:
        """Initialize the PGA autofocus processor.

        Args:
            max_iter: Maximum number of PGA iterations. Typically 3-5
                iterations are sufficient for convergence.
            convergence_threshold: Convergence criterion. Iteration stops when
                the RMS change in the estimated phase error between successive
                iterations falls below this threshold (in radians).
            window_type: Type of window to apply in step 2. Options:
                'hamming', 'hann', 'adaptive'. 'adaptive' uses a data-driven
                window based on the signal magnitude profile.
            window_fraction: Fraction of the azimuth aperture to include
                in the window (0.0 to 1.0). Smaller values focus on stronger
                scatterers but may introduce bias.
        """
        self.max_iter = max_iter
        self.convergence_threshold = convergence_threshold
        self.window_type = window_type
        self.window_fraction = window_fraction

        # Store diagnostics
        self.phase_errors: list[np.ndarray] = []
        self.convergence_history: list[float] = []

    def apply(
        self,
        image_complex: np.ndarray,
        axis: int = 1,
    ) -> np.ndarray:
        """Apply PGA autofocus to a complex SAR image.

        The input image should be in the spatial domain (after back-projection
        or matched filtering). PGA transforms to the azimuth-frequency domain,
        estimates phase errors, and corrects them.

        For back-projection images, PGA operates differently: it works on the
        raw phase history data. However, a common practical approach is to
        apply PGA on the image-domain data by treating each range bin as an
        independent 1D signal along the cross-range (azimuth) direction.

        Args:
            image_complex: 2D complex SAR image of shape (num_range, num_azimuth).
                Rows are range lines, columns are azimuth samples.
            axis: The axis along which to apply autofocus (azimuth direction).
                Default is 1 (columns).

        Returns:
            Phase-corrected complex SAR image of the same shape.
        """
        if image_complex.ndim != 2:
            raise ValueError(
                f"Expected 2D complex image, got shape {image_complex.shape}"
            )

        # Ensure we operate along azimuth (axis=1)
        if axis == 0:
            image_complex = image_complex.T

        num_range_bins, num_azimuth = image_complex.shape

        # Transform to azimuth-frequency domain
        # Each range bin is FFT'd along azimuth
        data_af = np.fft.fft(image_complex, axis=1)

        self.phase_errors = []
        self.convergence_history = []
        prev_phase_error = np.zeros(num_azimuth)

        for iteration in range(self.max_iter):
            # Step 1: Circular shift to center the dominant scatterer
            shifted_data = self._center_shift(data_af)

            # Step 2: Window the data
            windowed_data = self._apply_window(shifted_data)

            # Step 3: Estimate phase gradient (ML estimator)
            phase_gradient = self._estimate_phase_gradient(windowed_data)

            # Step 4: Integrate phase gradient to get phase error
            phase_error = self._integrate_phase_gradient(phase_gradient)

            # Remove linear trend (bulk motion, not an error)
            phase_error = self._remove_linear_trend(phase_error)

            self.phase_errors.append(phase_error.copy())

            # Step 5: Apply phase correction
            correction = np.exp(-1j * phase_error)
            data_af = data_af * correction[np.newaxis, :]

            # Check convergence
            delta = np.sqrt(np.mean((phase_error - prev_phase_error) ** 2))
            rms_error = np.sqrt(np.mean(phase_error ** 2))
            self.convergence_history.append(rms_error)

            if rms_error < self.convergence_threshold:
                break

            prev_phase_error = phase_error

        # Transform back to spatial domain
        corrected_image = np.fft.ifft(data_af, axis=1)

        if axis == 0:
            corrected_image = corrected_image.T

        return corrected_image

    def _center_shift(self, data: np.ndarray) -> np.ndarray:
        """Circularly shift each range line to center the brightest target.

        This aligns the dominant scatterer across range lines, ensuring that
        the phase error is coherent and can be estimated consistently.

        Args:
            data: Azimuth-frequency domain data, shape (num_range, num_azimuth).

        Returns:
            Shifted data with the same shape.
        """
        num_range_bins, num_azimuth = data.shape
        shifted = np.zeros_like(data)

        for m in range(num_range_bins):
            # Transform this range line to spatial domain
            spatial = np.fft.ifft(data[m, :])
            # Find the peak
            peak_idx = np.argmax(np.abs(spatial))
            # Circular shift to center
            shift_amount = num_azimuth // 2 - peak_idx
            shifted[m, :] = np.roll(data[m, :], shift_amount)

        return shifted

    def _apply_window(self, data: np.ndarray) -> np.ndarray:
        """Apply a window to emphasize the strong scatterer region.

        The window suppresses noise and weak scatterers that would degrade
        the phase gradient estimate.

        Args:
            data: Shifted azimuth-frequency data, shape (num_range, num_azimuth).

        Returns:
            Windowed data.
        """
        num_range_bins, num_azimuth = data.shape

        if self.window_type == "adaptive":
            # Data-driven window based on average magnitude profile
            avg_profile = np.mean(np.abs(np.fft.ifft(data, axis=1)), axis=0)
            peak_val = np.max(avg_profile)
            # Window width: region above threshold (e.g., -6 dB from peak)
            threshold = peak_val * 0.25  # -12 dB
            mask = avg_profile > threshold
            # Smooth the mask into a window
            win = np.zeros(num_azimuth)
            win[mask] = 1.0
            # Apply Hann taper to edges
            kernel_size = max(5, num_azimuth // 50)
            kernel = windows.hann(kernel_size * 2 + 1)
            win = np.convolve(win, kernel / kernel.sum(), mode="same")
            win = np.clip(win, 0, 1)
        else:
            # Fixed window centered in the aperture
            win_len = max(1, int(num_azimuth * self.window_fraction))
            pad_left = (num_azimuth - win_len) // 2
            pad_right = num_azimuth - win_len - pad_left

            if self.window_type == "hamming":
                win_core = windows.hamming(win_len)
            elif self.window_type == "hann":
                win_core = windows.hann(win_len)
            else:
                win_core = np.ones(win_len)

            win = np.concatenate([
                np.zeros(pad_left),
                win_core,
                np.zeros(pad_right),
            ])

        # Apply window to each range line
        return data * win[np.newaxis, :]

    def _estimate_phase_gradient(self, data: np.ndarray) -> np.ndarray:
        """Estimate the phase gradient using the ML estimator.

        The Maximum Likelihood estimator for the phase gradient is:

            phi'(n) = Im{ sum_m [ s_m(n) * conj(s_m(n-1)) ] }
                     / sum_m [ |s_m(n-1)|^2 ]

        This is equivalent to the weighted average of the instantaneous
        frequency across all range lines.

        Args:
            data: Windowed azimuth-frequency data, shape (num_range, num_azimuth).

        Returns:
            Phase gradient array of shape (num_azimuth,).
        """
        num_azimuth = data.shape[1]

        # Cross-correlation between adjacent azimuth samples
        # sum over range lines (axis=0)
        numerator = np.sum(data[:, 1:] * np.conj(data[:, :-1]), axis=0)
        denominator = np.sum(np.abs(data[:, :-1]) ** 2, axis=0)

        # Avoid division by zero
        denominator = np.where(denominator > 1e-30, denominator, 1e-30)

        # Phase gradient
        phase_gradient = np.zeros(num_azimuth)
        phase_gradient[1:] = np.imag(numerator) / denominator

        return phase_gradient

    def _integrate_phase_gradient(self, phase_gradient: np.ndarray) -> np.ndarray:
        """Integrate the phase gradient to obtain the absolute phase error.

        Simple cumulative sum integration:
            phi(n) = sum_{k=0}^{n} phi'(k)

        Args:
            phase_gradient: Phase gradient array of shape (num_azimuth,).

        Returns:
            Integrated phase error of shape (num_azimuth,).
        """
        return np.cumsum(phase_gradient)

    def _remove_linear_trend(self, phase_error: np.ndarray) -> np.ndarray:
        """Remove the linear component from the phase error.

        A linear phase error corresponds to a bulk position shift of the
        image and is not a true focusing error. Removing it prevents
        unnecessary image shifting.

        Args:
            phase_error: Phase error array of shape (num_azimuth,).

        Returns:
            Phase error with linear trend removed.
        """
        n = len(phase_error)
        x = np.arange(n, dtype=np.float64)

        # Least-squares linear fit: phi(n) = a*n + b
        x_mean = x.mean()
        phi_mean = phase_error.mean()
        a = np.sum((x - x_mean) * (phase_error - phi_mean)) / np.sum((x - x_mean) ** 2)
        b = phi_mean - a * x_mean

        linear_trend = a * x + b
        return phase_error - linear_trend

    def get_total_phase_correction(self) -> np.ndarray:
        """Get the cumulative phase correction applied across all iterations.

        Returns:
            Total phase error estimate (sum of all iteration corrections).
        """
        if not self.phase_errors:
            return np.array([])
        total = np.zeros_like(self.phase_errors[0])
        for pe in self.phase_errors:
            total += pe
        return total

    def get_convergence_history(self) -> list[float]:
        """Return the RMS phase error at each iteration.

        Returns:
            List of RMS phase error values (in radians), one per iteration.
        """
        return self.convergence_history
