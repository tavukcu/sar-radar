"""
Data Loading Utilities

Provides functions to load raw radar data from various sources:
    - DCA1000EVM binary format (raw 16-bit ADC samples)
    - IMU sensor data from CSV / binary log files
    - HDF5 archives containing complete radar + IMU recordings

The DCA1000EVM is TI's real-time data capture adapter for the AWR2243. It
streams raw ADC samples over Ethernet as interleaved 16-bit signed integers
in the order: [RX1_I, RX1_Q, RX2_I, RX2_Q, ...] for each sample of each chirp.

For SD-card recordings from our custom hardware, we use HDF5 files with
datasets for radar data, IMU data, timestamps, and metadata.
"""

from __future__ import annotations

import csv
import struct
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

try:
    import h5py

    HAS_H5PY = True
except ImportError:
    HAS_H5PY = False

from sar_processor.radar_config import RadarConfig


def load_raw_binary(
    filepath: str | Path,
    config: RadarConfig,
    num_frames: int = 1,
    complex_data: bool = True,
) -> np.ndarray:
    """Load raw ADC data from DCA1000EVM binary capture file.

    The DCA1000EVM stores interleaved 16-bit signed integer samples in the
    following order per sample clock:
        [RX0_I, RX0_Q, RX1_I, RX1_Q, ..., RXn_I, RXn_Q]

    For real-only ADC mode (complex_data=False), the interleaving is:
        [RX0, RX1, ..., RXn]

    Args:
        filepath: Path to the .bin raw data file.
        config: RadarConfig describing the chirp parameters.
        num_frames: Number of complete frames to load. If -1, load all available.
        complex_data: If True, the ADC outputs complex (I/Q) samples.
            The AWR2243 typically outputs complex baseband data.

    Returns:
        Complex numpy array of shape:
            (num_frames, num_chirps, num_rx, num_samples) if complex_data=True
            (num_frames, num_chirps, num_rx, num_samples) real if complex_data=False

    Raises:
        FileNotFoundError: If the binary file does not exist.
        ValueError: If the file size is inconsistent with the configuration.
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Raw binary file not found: {filepath}")

    # Read raw bytes
    raw_bytes = filepath.read_bytes()
    raw_int16 = np.frombuffer(raw_bytes, dtype=np.int16)

    if complex_data:
        # Each sample consists of I and Q for each RX channel
        samples_per_chirp = config.num_samples * config.num_rx * 2  # *2 for I/Q
    else:
        samples_per_chirp = config.num_samples * config.num_rx

    samples_per_frame = samples_per_chirp * config.num_chirps
    total_frames_available = len(raw_int16) // samples_per_frame

    if num_frames == -1:
        num_frames = total_frames_available
    elif num_frames > total_frames_available:
        print(
            f"Warning: Requested {num_frames} frames but only "
            f"{total_frames_available} available. Loading all."
        )
        num_frames = total_frames_available

    if num_frames == 0:
        raise ValueError(
            f"File too small for even one complete frame. "
            f"Expected at least {samples_per_frame * 2} bytes, "
            f"got {len(raw_bytes)} bytes."
        )

    # Trim to exact number of complete frames
    total_samples = num_frames * samples_per_frame
    raw_int16 = raw_int16[:total_samples]

    if complex_data:
        # Reshape: (frames, chirps, rx_channels * 2, samples) then combine I/Q
        raw_int16 = raw_int16.reshape(
            num_frames, config.num_chirps, config.num_rx * 2, config.num_samples
        )
        # Separate I and Q channels: even indices are I, odd indices are Q
        data_i = raw_int16[:, :, 0::2, :].astype(np.float64)
        data_q = raw_int16[:, :, 1::2, :].astype(np.float64)
        data = data_i + 1j * data_q
    else:
        data = raw_int16.reshape(
            num_frames, config.num_chirps, config.num_rx, config.num_samples
        ).astype(np.float64)

    return data


def load_imu_data(
    filepath: str | Path,
    delimiter: str = ",",
) -> dict:
    """Load IMU sensor data from a CSV or binary log file.

    Expected CSV columns:
        timestamp_s, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z [, mag_x, mag_y, mag_z]

    Accelerometer values are in m/s^2, gyroscope in rad/s, magnetometer in uT.
    Timestamps are in seconds (can be relative to recording start).

    For binary format (.bin), the file is assumed to contain packed structs:
        7 x float64 per record (timestamp, ax, ay, az, gx, gy, gz)

    Args:
        filepath: Path to the IMU data file (.csv or .bin).
        delimiter: CSV delimiter character. Default is comma.

    Returns:
        Dictionary with keys:
            - 'timestamps': (N,) array of timestamps in seconds
            - 'acc': (N, 3) array of accelerometer data [ax, ay, az] in m/s^2
            - 'gyro': (N, 3) array of gyroscope data [gx, gy, gz] in rad/s
            - 'mag': (N, 3) array of magnetometer data or None if not present

    Raises:
        FileNotFoundError: If the IMU data file does not exist.
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"IMU data file not found: {filepath}")

    if filepath.suffix == ".bin":
        return _load_imu_binary(filepath)

    # CSV loading
    timestamps = []
    acc_data = []
    gyro_data = []
    mag_data = []
    has_mag = False

    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f, delimiter=delimiter)

        # Skip header if present
        first_row = next(reader)
        try:
            float(first_row[0])
            # First row is data, not header
            _parse_imu_row(first_row, timestamps, acc_data, gyro_data, mag_data)
            if len(first_row) >= 10:
                has_mag = True
        except ValueError:
            # First row is a header
            has_mag = len(first_row) >= 10

        for row in reader:
            if len(row) < 7:
                continue
            _parse_imu_row(row, timestamps, acc_data, gyro_data, mag_data)

    result = {
        "timestamps": np.array(timestamps, dtype=np.float64),
        "acc": np.array(acc_data, dtype=np.float64),
        "gyro": np.array(gyro_data, dtype=np.float64),
        "mag": np.array(mag_data, dtype=np.float64) if has_mag and mag_data else None,
    }
    return result


def _parse_imu_row(
    row: list,
    timestamps: list,
    acc_data: list,
    gyro_data: list,
    mag_data: list,
) -> None:
    """Parse a single CSV row into the IMU data lists."""
    timestamps.append(float(row[0]))
    acc_data.append([float(row[1]), float(row[2]), float(row[3])])
    gyro_data.append([float(row[4]), float(row[5]), float(row[6])])
    if len(row) >= 10:
        mag_data.append([float(row[7]), float(row[8]), float(row[9])])


def _load_imu_binary(filepath: Path) -> dict:
    """Load IMU data from a packed binary file.

    Binary format: consecutive records of 7 x float64 each
    (timestamp, ax, ay, az, gx, gy, gz).
    """
    raw = filepath.read_bytes()
    record_size = 7 * 8  # 7 doubles, 8 bytes each
    num_records = len(raw) // record_size

    data = np.frombuffer(raw[: num_records * record_size], dtype=np.float64)
    data = data.reshape(num_records, 7)

    return {
        "timestamps": data[:, 0],
        "acc": data[:, 1:4],
        "gyro": data[:, 4:7],
        "mag": None,
    }


def load_hdf5(filepath: str | Path) -> dict:
    """Load a complete SAR recording from an HDF5 file.

    Our custom HDF5 format stores radar data, IMU data, and metadata in a
    single archive for convenient storage on the SD card and post-processing.

    Expected HDF5 structure:
        /radar_data         - Complex array (num_chirps, num_rx, num_samples)
        /imu/timestamps     - IMU timestamp array
        /imu/accelerometer  - Accelerometer data (N, 3)
        /imu/gyroscope      - Gyroscope data (N, 3)
        /imu/magnetometer   - (optional) Magnetometer data (N, 3)
        /config             - Group with radar config attributes
        /metadata           - Group with recording metadata

    Args:
        filepath: Path to the HDF5 file.

    Returns:
        Dictionary containing:
            - 'radar_data': complex ndarray of raw or range-compressed data
            - 'imu': dict with 'timestamps', 'acc', 'gyro', 'mag' arrays
            - 'config': RadarConfig instance
            - 'metadata': dict of recording metadata

    Raises:
        ImportError: If h5py is not installed.
        FileNotFoundError: If the HDF5 file does not exist.
    """
    if not HAS_H5PY:
        raise ImportError("h5py is required to load HDF5 files. Install with: pip install h5py")

    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"HDF5 file not found: {filepath}")

    result = {}

    with h5py.File(filepath, "r") as hf:
        # Load radar data
        if "radar_data" in hf:
            result["radar_data"] = hf["radar_data"][:]

        # Load IMU data
        imu = {}
        if "imu" in hf:
            imu_grp = hf["imu"]
            imu["timestamps"] = imu_grp["timestamps"][:] if "timestamps" in imu_grp else None
            imu["acc"] = imu_grp["accelerometer"][:] if "accelerometer" in imu_grp else None
            imu["gyro"] = imu_grp["gyroscope"][:] if "gyroscope" in imu_grp else None
            imu["mag"] = imu_grp["magnetometer"][:] if "magnetometer" in imu_grp else None
        result["imu"] = imu

        # Load radar config
        if "config" in hf:
            cfg_grp = hf["config"]
            config_dict = {key: cfg_grp.attrs[key] for key in cfg_grp.attrs}
            # Convert numpy scalars to Python types for dataclass
            for k, v in config_dict.items():
                if hasattr(v, "item"):
                    config_dict[k] = v.item()
            result["config"] = RadarConfig(**config_dict)
        else:
            result["config"] = None

        # Load metadata
        metadata = {}
        if "metadata" in hf:
            meta_grp = hf["metadata"]
            metadata = {key: meta_grp.attrs[key] for key in meta_grp.attrs}
            # Decode bytes to strings
            for k, v in metadata.items():
                if isinstance(v, bytes):
                    metadata[k] = v.decode("utf-8")
        result["metadata"] = metadata

        # Load processed SAR image if present
        if "sar_image" in hf:
            result["sar_image"] = hf["sar_image"][:]
        if "antenna_positions" in hf:
            result["antenna_positions"] = hf["antenna_positions"][:]

    return result


def save_hdf5(
    filepath: str | Path,
    radar_data: np.ndarray,
    imu_data: Optional[dict] = None,
    config: Optional[RadarConfig] = None,
    sar_image: Optional[np.ndarray] = None,
    antenna_positions: Optional[np.ndarray] = None,
    metadata: Optional[dict] = None,
) -> None:
    """Save SAR recording and processing results to an HDF5 file.

    Args:
        filepath: Output HDF5 file path.
        radar_data: Raw or range-compressed radar data array.
        imu_data: Dictionary with 'timestamps', 'acc', 'gyro', 'mag' arrays.
        config: RadarConfig instance to serialize.
        sar_image: Processed SAR image (complex or magnitude).
        antenna_positions: Estimated antenna trajectory (N, 3) array.
        metadata: Additional metadata key-value pairs.
    """
    if not HAS_H5PY:
        raise ImportError("h5py is required to save HDF5 files. Install with: pip install h5py")

    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(filepath, "w") as hf:
        # Radar data
        hf.create_dataset(
            "radar_data",
            data=radar_data,
            compression="gzip",
            compression_opts=4,
        )

        # IMU data
        if imu_data is not None:
            imu_grp = hf.create_group("imu")
            if imu_data.get("timestamps") is not None:
                imu_grp.create_dataset("timestamps", data=imu_data["timestamps"])
            if imu_data.get("acc") is not None:
                imu_grp.create_dataset("accelerometer", data=imu_data["acc"])
            if imu_data.get("gyro") is not None:
                imu_grp.create_dataset("gyroscope", data=imu_data["gyro"])
            if imu_data.get("mag") is not None:
                imu_grp.create_dataset("magnetometer", data=imu_data["mag"])

        # Radar configuration
        if config is not None:
            cfg_grp = hf.create_group("config")
            config_dict = config.to_dict()
            for key, value in config_dict.items():
                if value is not None:
                    cfg_grp.attrs[key] = value

        # Processed SAR image
        if sar_image is not None:
            hf.create_dataset(
                "sar_image",
                data=sar_image,
                compression="gzip",
                compression_opts=4,
            )

        # Antenna positions / trajectory
        if antenna_positions is not None:
            hf.create_dataset("antenna_positions", data=antenna_positions)

        # Metadata
        if metadata is not None:
            meta_grp = hf.create_group("metadata")
            for key, value in metadata.items():
                if isinstance(value, str):
                    meta_grp.attrs[key] = value.encode("utf-8")
                else:
                    meta_grp.attrs[key] = value
