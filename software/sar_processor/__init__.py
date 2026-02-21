"""
SAR Processor - 77 GHz Handheld SAR Radar Image Processing Pipeline

This package provides a complete SAR (Synthetic Aperture Radar) processing
pipeline for a handheld 77 GHz FMCW radar system based on the TI AWR2243.

Main components:
    - BackProjectionAlgorithm: Time-domain back-projection for SAR image formation
    - PhaseGradientAutofocus: Iterative PGA for phase error correction
    - MotionCompensator: IMU-based motion compensation with AHRS fusion
    - RadarConfig: Radar system configuration parameters
    - DataLoader: Raw data loading from DCA1000EVM / HDF5 / binary formats

Typical usage:
    from sar_processor import (
        BackProjectionAlgorithm,
        PhaseGradientAutofocus,
        MotionCompensator,
        RadarConfig,
    )

    config = RadarConfig.default()
    bpa = BackProjectionAlgorithm(config)
    rc_data = bpa.range_compression(raw_data)
    image = bpa.backproject(rc_data, antenna_positions)
"""

from sar_processor.radar_config import RadarConfig
from sar_processor.bpa import BackProjectionAlgorithm
from sar_processor.autofocus import PhaseGradientAutofocus
from sar_processor.motion_comp import MotionCompensator
from sar_processor.data_loader import load_raw_binary, load_imu_data, load_hdf5, save_hdf5

__version__ = "0.1.0"
__author__ = "SAR Radar Project"

__all__ = [
    "RadarConfig",
    "BackProjectionAlgorithm",
    "PhaseGradientAutofocus",
    "MotionCompensator",
    "load_raw_binary",
    "load_imu_data",
    "load_hdf5",
    "save_hdf5",
]
