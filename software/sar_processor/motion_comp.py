"""
Motion Compensation using IMU Data

Provides trajectory estimation and phase correction for handheld SAR radar
using inertial measurement unit (IMU) data. Accurate motion compensation is
critical for handheld SAR because the antenna trajectory is irregular and
unknown a priori.

## Approach

1. **Attitude Estimation**: Fuse accelerometer, gyroscope, and (optionally)
   magnetometer data using a complementary filter (Madgwick or Mahony) from
   the AHRS library to estimate the orientation (quaternion) of the device
   at each IMU sample.

2. **Gravity Removal**: Use the estimated orientation to rotate the
   accelerometer measurements from the body frame to the world (navigation)
   frame, then subtract gravity.

3. **Position Integration**: Double-integrate the world-frame acceleration
   to obtain velocity and position. Apply Zero-Velocity Updates (ZUPT) to
   bound the integration drift when the device is detected as stationary.

4. **Trajectory Interpolation**: Resample the IMU-derived trajectory to
   match the radar pulse timestamps (PRI grid).

5. **Phase Correction**: For each radar pulse, compute the deviation from
   an ideal straight-line path and apply a compensating phase shift:

       phase_correction(n) = exp(-j * 4*pi*fc/c * delta_R(n))

   where delta_R(n) is the range deviation of the antenna from the ideal path.

## Limitations

   - Pure IMU integration drifts over time; ZUPT helps but does not eliminate
     all drift for long scans.
   - The PGA autofocus stage is expected to correct residual phase errors.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt

try:
    import ahrs
    from ahrs.filters import Madgwick, Mahony

    HAS_AHRS = True
except ImportError:
    HAS_AHRS = False

from sar_processor.radar_config import RadarConfig, C


class MotionCompensator:
    """IMU-based motion compensation for handheld SAR.

    Estimates the antenna trajectory from IMU measurements and applies
    phase corrections to the range-compressed radar data.

    Usage:
        mc = MotionCompensator(imu_data, radar_timestamps, config)
        trajectory = mc.estimate_trajectory()
        positions = mc.get_antenna_positions()
        corrected_data = mc.compensate(rc_data, trajectory)
    """

    def __init__(
        self,
        imu_data: dict,
        radar_timestamps: np.ndarray,
        config: RadarConfig,
        filter_type: str = "madgwick",
        zupt_threshold: float = 0.5,
        zupt_window: int = 10,
        acc_bias: Optional[np.ndarray] = None,
        gyro_bias: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize the motion compensator.

        Args:
            imu_data: Dictionary containing IMU data with keys:
                - 'timestamps': (N,) array of IMU timestamps in seconds
                - 'acc': (N, 3) array of accelerometer data [ax, ay, az] in m/s^2
                - 'gyro': (N, 3) array of gyroscope data [gx, gy, gz] in rad/s
                - 'mag': (N, 3) array of magnetometer data or None
            radar_timestamps: (M,) array of radar pulse timestamps in seconds.
                Typically generated from PRI: t_n = n * PRI.
            config: Radar configuration parameters.
            filter_type: AHRS filter to use: 'madgwick' or 'mahony'.
            zupt_threshold: Acceleration magnitude threshold (m/s^2) for
                zero-velocity update detection. When the acceleration norm
                is within [g - threshold, g + threshold] for zupt_window
                consecutive samples, the device is assumed stationary.
            zupt_window: Number of consecutive samples below threshold to
                trigger a zero-velocity update.
            acc_bias: Optional (3,) accelerometer bias vector to subtract.
            gyro_bias: Optional (3,) gyroscope bias vector to subtract.
        """
        self.imu_timestamps = imu_data["timestamps"].copy()
        self.acc_raw = imu_data["acc"].copy()
        self.gyro_raw = imu_data["gyro"].copy()
        self.mag_raw = imu_data["mag"].copy() if imu_data.get("mag") is not None else None
        self.radar_timestamps = radar_timestamps.copy()
        self.config = config
        self.filter_type = filter_type
        self.zupt_threshold = zupt_threshold
        self.zupt_window = zupt_window

        # Apply bias corrections
        if acc_bias is not None:
            self.acc_raw -= acc_bias[np.newaxis, :]
        if gyro_bias is not None:
            self.gyro_raw -= gyro_bias[np.newaxis, :]

        # IMU sample rate
        dt_imu = np.median(np.diff(self.imu_timestamps))
        self.imu_fs = 1.0 / dt_imu

        # Estimated quantities (populated by estimate_trajectory)
        self.quaternions: Optional[np.ndarray] = None
        self.acc_world: Optional[np.ndarray] = None
        self.velocity: Optional[np.ndarray] = None
        self.position: Optional[np.ndarray] = None
        self.antenna_positions: Optional[np.ndarray] = None

    def estimate_trajectory(self) -> np.ndarray:
        """Estimate the full 3D trajectory from IMU data.

        Performs attitude estimation, gravity removal, ZUPT detection, and
        double integration to produce position estimates at each IMU sample,
        then interpolates to radar pulse timestamps.

        Returns:
            Position array of shape (num_radar_pulses, 3) in meters,
            in the world (navigation) coordinate frame.
        """
        num_imu_samples = len(self.imu_timestamps)
        dt = 1.0 / self.imu_fs

        # ------------------------------------------------------------ #
        # Step 1: Attitude estimation using AHRS filter
        # ------------------------------------------------------------ #
        self.quaternions = self._estimate_attitude()

        # ------------------------------------------------------------ #
        # Step 2: Rotate accelerometer to world frame & remove gravity
        # ------------------------------------------------------------ #
        self.acc_world = self._remove_gravity(self.acc_raw, self.quaternions)

        # ------------------------------------------------------------ #
        # Step 3: Detect zero-velocity intervals (ZUPT)
        # ------------------------------------------------------------ #
        zupt_mask = self._detect_zupt(self.acc_raw)

        # ------------------------------------------------------------ #
        # Step 4: High-pass filter acceleration to reduce drift
        # ------------------------------------------------------------ #
        self.acc_world = self._highpass_filter(self.acc_world, cutoff=0.1, fs=self.imu_fs)

        # ------------------------------------------------------------ #
        # Step 5: Integrate acceleration -> velocity -> position
        # ------------------------------------------------------------ #
        self.velocity = np.zeros((num_imu_samples, 3))
        self.position = np.zeros((num_imu_samples, 3))

        for i in range(1, num_imu_samples):
            # Trapezoidal integration for velocity
            self.velocity[i] = self.velocity[i - 1] + \
                0.5 * (self.acc_world[i] + self.acc_world[i - 1]) * dt

            # Apply ZUPT: reset velocity to zero when stationary
            if zupt_mask[i]:
                self.velocity[i] = 0.0

            # Trapezoidal integration for position
            self.position[i] = self.position[i - 1] + \
                0.5 * (self.velocity[i] + self.velocity[i - 1]) * dt

        # ------------------------------------------------------------ #
        # Step 6: Interpolate to radar pulse timestamps
        # ------------------------------------------------------------ #
        self.antenna_positions = self._interpolate_to_radar_times(
            self.imu_timestamps, self.position, self.radar_timestamps
        )

        return self.antenna_positions

    def _estimate_attitude(self) -> np.ndarray:
        """Estimate device orientation (quaternions) using AHRS filter.

        Uses the Madgwick or Mahony complementary filter to fuse accelerometer,
        gyroscope, and optional magnetometer data into a quaternion attitude
        estimate at each IMU sample.

        Returns:
            Quaternion array of shape (N, 4) in [w, x, y, z] convention.
        """
        num_samples = len(self.imu_timestamps)
        dt = 1.0 / self.imu_fs

        if HAS_AHRS:
            if self.filter_type == "madgwick":
                filt = Madgwick(
                    gyr=self.gyro_raw,
                    acc=self.acc_raw,
                    mag=self.mag_raw,
                    frequency=self.imu_fs,
                )
            elif self.filter_type == "mahony":
                filt = Mahony(
                    gyr=self.gyro_raw,
                    acc=self.acc_raw,
                    mag=self.mag_raw,
                    frequency=self.imu_fs,
                )
            else:
                raise ValueError(f"Unknown filter type: {self.filter_type}")

            return filt.Q
        else:
            # Fallback: simple gyroscope integration (no fusion)
            print(
                "Warning: AHRS library not available. Using gyroscope-only "
                "integration (less accurate)."
            )
            return self._gyro_only_attitude(self.gyro_raw, dt)

    def _gyro_only_attitude(self, gyro: np.ndarray, dt: float) -> np.ndarray:
        """Simple quaternion integration from gyroscope data only.

        This is a fallback when the AHRS library is not available. It does
        not correct for gyroscope drift.

        Args:
            gyro: Gyroscope data (N, 3) in rad/s.
            dt: Time step in seconds.

        Returns:
            Quaternion array (N, 4) in [w, x, y, z] convention.
        """
        num_samples = gyro.shape[0]
        q = np.zeros((num_samples, 4))
        q[0] = [1.0, 0.0, 0.0, 0.0]  # Identity quaternion

        for i in range(1, num_samples):
            wx, wy, wz = gyro[i]
            omega_norm = np.sqrt(wx**2 + wy**2 + wz**2)

            if omega_norm > 1e-10:
                half_angle = 0.5 * omega_norm * dt
                s = np.sin(half_angle) / omega_norm
                dq = np.array([np.cos(half_angle), s * wx, s * wy, s * wz])
            else:
                dq = np.array([1.0, 0.0, 0.0, 0.0])

            q[i] = self._quaternion_multiply(q[i - 1], dq)
            q[i] /= np.linalg.norm(q[i])

        return q

    @staticmethod
    def _quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Hamilton product of two quaternions [w, x, y, z]."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ])

    @staticmethod
    def _quaternion_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Rotate vector v by quaternion q.

        Computes q * [0, v] * q_conj using the rotation matrix derived from q.

        Args:
            q: Quaternion [w, x, y, z].
            v: 3D vector [x, y, z].

        Returns:
            Rotated 3D vector.
        """
        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)],
        ])

        return R @ v

    def _remove_gravity(
        self,
        acc_body: np.ndarray,
        quaternions: np.ndarray,
    ) -> np.ndarray:
        """Rotate acceleration to world frame and subtract gravity.

        Args:
            acc_body: Body-frame acceleration (N, 3) in m/s^2.
            quaternions: Attitude quaternions (N, 4).

        Returns:
            World-frame acceleration with gravity removed, shape (N, 3).
        """
        gravity_world = np.array([0.0, 0.0, 9.81])  # m/s^2 downward in NED
        num_samples = acc_body.shape[0]
        acc_world = np.zeros_like(acc_body)

        for i in range(num_samples):
            # Rotate body-frame acceleration to world frame
            acc_world[i] = self._quaternion_rotate(quaternions[i], acc_body[i])
            # Subtract gravity
            acc_world[i] -= gravity_world

        return acc_world

    def _detect_zupt(self, acc_body: np.ndarray) -> np.ndarray:
        """Detect zero-velocity (stationary) intervals using accelerometer.

        When the device is stationary, the accelerometer should read only
        gravity (~9.81 m/s^2). We detect this by checking if the acceleration
        magnitude is close to g for several consecutive samples.

        Args:
            acc_body: Body-frame acceleration (N, 3) in m/s^2.

        Returns:
            Boolean mask (N,) where True indicates a stationary sample.
        """
        g = 9.81
        acc_norm = np.linalg.norm(acc_body, axis=1)
        near_gravity = np.abs(acc_norm - g) < self.zupt_threshold

        # Require zupt_window consecutive samples near gravity
        zupt_mask = np.zeros(len(acc_norm), dtype=bool)
        count = 0
        for i in range(len(near_gravity)):
            if near_gravity[i]:
                count += 1
            else:
                count = 0

            if count >= self.zupt_window:
                zupt_mask[i - self.zupt_window + 1: i + 1] = True

        return zupt_mask

    @staticmethod
    def _highpass_filter(
        data: np.ndarray,
        cutoff: float = 0.1,
        fs: float = 100.0,
        order: int = 2,
    ) -> np.ndarray:
        """Apply a high-pass Butterworth filter to remove low-frequency drift.

        Args:
            data: Input data array, shape (N, 3) or (N,).
            cutoff: Cutoff frequency in Hz.
            fs: Sample rate in Hz.
            order: Filter order.

        Returns:
            Filtered data with the same shape.
        """
        nyq = 0.5 * fs
        if cutoff >= nyq:
            return data
        b, a = butter(order, cutoff / nyq, btype="high")
        if data.ndim == 1:
            return filtfilt(b, a, data)
        else:
            filtered = np.zeros_like(data)
            for col in range(data.shape[1]):
                filtered[:, col] = filtfilt(b, a, data[:, col])
            return filtered

    @staticmethod
    def _interpolate_to_radar_times(
        imu_times: np.ndarray,
        imu_positions: np.ndarray,
        radar_times: np.ndarray,
    ) -> np.ndarray:
        """Interpolate IMU-derived positions to radar pulse timestamps.

        Args:
            imu_times: IMU timestamp array (N,).
            imu_positions: Position array (N, 3).
            radar_times: Radar pulse timestamps (M,).

        Returns:
            Interpolated positions at radar times, shape (M, 3).
        """
        positions_interp = np.zeros((len(radar_times), 3))

        for axis in range(3):
            interp_func = interp1d(
                imu_times,
                imu_positions[:, axis],
                kind="cubic",
                bounds_error=False,
                fill_value="extrapolate",
            )
            positions_interp[:, axis] = interp_func(radar_times)

        return positions_interp

    def compensate(
        self,
        range_compressed_data: np.ndarray,
        trajectory: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Apply motion compensation to range-compressed radar data.

        For each pulse, compute the deviation of the actual antenna position
        from an ideal straight-line path, then apply a phase correction:

            corrected(n) = rc_data(n) * exp(-j * 4*pi*fc/c * delta_R(n))

        where delta_R(n) is the range error (component of the position
        deviation along the line-of-sight direction).

        Args:
            range_compressed_data: Range-compressed data of shape
                (num_pulses, num_range_bins). Can also be
                (num_pulses, num_rx, num_range_bins).
            trajectory: Optional (num_pulses, 3) antenna positions. If None,
                uses the trajectory from the most recent estimate_trajectory().

        Returns:
            Motion-compensated range-compressed data with the same shape.
        """
        if trajectory is None:
            if self.antenna_positions is None:
                raise RuntimeError(
                    "No trajectory available. Call estimate_trajectory() first "
                    "or provide a trajectory argument."
                )
            trajectory = self.antenna_positions

        num_pulses = range_compressed_data.shape[0]
        if num_pulses != trajectory.shape[0]:
            raise ValueError(
                f"Number of pulses ({num_pulses}) does not match "
                f"trajectory length ({trajectory.shape[0]})"
            )

        # Compute ideal linear path (straight line from start to end)
        ideal_path = np.linspace(
            trajectory[0], trajectory[-1], num_pulses
        )

        # Position deviation from ideal path
        deviation = trajectory - ideal_path

        # Range deviation: project deviation onto the look direction.
        # For a forward-looking radar, the primary look direction is +y.
        # More generally, use the instantaneous look direction.
        # Simplified: use y-component (range direction) as the dominant term.
        # For full 3D: delta_R = magnitude of deviation along LOS.
        # Here we use the total displacement magnitude with sign from y-component.
        delta_r = np.linalg.norm(deviation, axis=1)
        # Assign sign based on whether deviation is toward or away from scene
        sign = np.sign(deviation[:, 1])  # y is range direction
        sign[sign == 0] = 1.0
        delta_r *= sign

        # Phase correction factor
        k = 4.0 * np.pi * self.config.fc / C
        phase_correction = np.exp(-1j * k * delta_r)

        # Apply correction
        if range_compressed_data.ndim == 2:
            corrected = range_compressed_data * phase_correction[:, np.newaxis]
        elif range_compressed_data.ndim == 3:
            corrected = range_compressed_data * phase_correction[:, np.newaxis, np.newaxis]
        else:
            raise ValueError(
                f"Unexpected data shape: {range_compressed_data.shape}"
            )

        return corrected

    def get_antenna_positions(self) -> np.ndarray:
        """Get the estimated antenna positions at radar pulse times.

        Returns:
            Position array of shape (num_pulses, 3) in meters.

        Raises:
            RuntimeError: If estimate_trajectory() has not been called yet.
        """
        if self.antenna_positions is None:
            raise RuntimeError(
                "Trajectory not yet estimated. Call estimate_trajectory() first."
            )
        return self.antenna_positions.copy()

    def get_velocity(self) -> Optional[np.ndarray]:
        """Get the estimated velocity at IMU sample times.

        Returns:
            Velocity array of shape (N, 3) in m/s, or None if not computed.
        """
        return self.velocity.copy() if self.velocity is not None else None

    @staticmethod
    def generate_ideal_trajectory(
        scan_length: float,
        num_pulses: int,
        scan_axis: str = "x",
        standoff_distance: float = 0.3,
    ) -> np.ndarray:
        """Generate an ideal linear scan trajectory for testing.

        Args:
            scan_length: Total scan length in meters.
            num_pulses: Number of radar pulses (positions).
            scan_axis: Axis along which to scan ('x' or 'y').
            standoff_distance: Distance from the scan line to the scene
                (in the range direction).

        Returns:
            Position array of shape (num_pulses, 3).
        """
        positions = np.zeros((num_pulses, 3))

        if scan_axis == "x":
            positions[:, 0] = np.linspace(
                -scan_length / 2, scan_length / 2, num_pulses
            )
            positions[:, 1] = standoff_distance
        elif scan_axis == "y":
            positions[:, 1] = np.linspace(
                -scan_length / 2, scan_length / 2, num_pulses
            )
            positions[:, 0] = standoff_distance
        else:
            raise ValueError(f"scan_axis must be 'x' or 'y', got '{scan_axis}'")

        return positions
