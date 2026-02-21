"""
SAR Processor - Main Entry Point

Command-line interface for the 77 GHz handheld SAR radar image processing
pipeline. Supports both offline (file-based) and live (WiFi streaming) modes.

Offline mode:
    python -m sar_processor.main --input recording.h5 --output result.h5

Live mode:
    python -m sar_processor.main --live --host 192.168.4.1 --port 5000

Usage:
    python -m sar_processor.main --help
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        prog="sar_processor",
        description=(
            "77 GHz Handheld SAR Radar - Image Processing Pipeline\n\n"
            "Performs SAR image formation using the Back-Projection Algorithm\n"
            "with IMU-based motion compensation and PGA autofocus."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # -- Input / Output --
    io_group = parser.add_argument_group("Input / Output")
    io_group.add_argument(
        "--input", "-i",
        type=str,
        default=None,
        help="Path to input data file (HDF5 .h5 or raw binary .bin).",
    )
    io_group.add_argument(
        "--imu",
        type=str,
        default=None,
        help="Path to IMU data file (CSV or binary). Required for .bin input.",
    )
    io_group.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Path for output results file (HDF5 .h5 or NumPy .npz).",
    )
    io_group.add_argument(
        "--config", "-c",
        type=str,
        default=None,
        help="Path to radar configuration JSON file. Uses defaults if not specified.",
    )

    # -- Processing Options --
    proc_group = parser.add_argument_group("Processing")
    proc_group.add_argument(
        "--gpu",
        action="store_true",
        default=False,
        help="Use GPU (CUDA) acceleration for back-projection.",
    )
    proc_group.add_argument(
        "--cpu",
        action="store_true",
        default=False,
        help="Force CPU processing (NumPy), even if GPU is available.",
    )
    proc_group.add_argument(
        "--no-autofocus",
        action="store_true",
        default=False,
        help="Skip the PGA autofocus step.",
    )
    proc_group.add_argument(
        "--no-mocomp",
        action="store_true",
        default=False,
        help="Skip motion compensation (use ideal linear trajectory).",
    )
    proc_group.add_argument(
        "--pga-iterations",
        type=int,
        default=5,
        help="Maximum number of PGA autofocus iterations (default: 5).",
    )

    # -- Image Grid --
    grid_group = parser.add_argument_group("Image Grid")
    grid_group.add_argument(
        "--x-extent",
        type=float,
        nargs=2,
        default=[-0.5, 0.5],
        metavar=("XMIN", "XMAX"),
        help="Cross-range extent in meters (default: -0.5 0.5).",
    )
    grid_group.add_argument(
        "--y-extent",
        type=float,
        nargs=2,
        default=[0.1, 1.0],
        metavar=("YMIN", "YMAX"),
        help="Range extent in meters (default: 0.1 1.0).",
    )
    grid_group.add_argument(
        "--pixel-size",
        type=float,
        default=0.005,
        help="Output image pixel size in meters (default: 0.005 = 5 mm).",
    )

    # -- Live Mode --
    live_group = parser.add_argument_group("Live Mode")
    live_group.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Enable live WiFi streaming mode.",
    )
    live_group.add_argument(
        "--host",
        type=str,
        default="192.168.4.1",
        help="Radar ESP32 IP address for live mode (default: 192.168.4.1).",
    )
    live_group.add_argument(
        "--port",
        type=int,
        default=5000,
        help="TCP port for live data streaming (default: 5000).",
    )

    # -- Display --
    display_group = parser.add_argument_group("Display")
    display_group.add_argument(
        "--gui",
        action="store_true",
        default=False,
        help="Launch the GUI application instead of CLI processing.",
    )
    display_group.add_argument(
        "--show",
        action="store_true",
        default=False,
        help="Display the resulting SAR image using matplotlib.",
    )
    display_group.add_argument(
        "--dynamic-range",
        type=float,
        default=60.0,
        help="Dynamic range for display in dB (default: 60).",
    )

    # -- Verbosity --
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        default=False,
        help="Enable verbose output.",
    )

    return parser.parse_args()


def process_offline(args: argparse.Namespace) -> None:
    """Run the offline (file-based) SAR processing pipeline.

    Steps:
        1. Load radar configuration
        2. Load raw radar data
        3. Load IMU data
        4. Range compression
        5. Motion compensation (trajectory estimation + phase correction)
        6. Back-projection (image formation)
        7. PGA autofocus
        8. Save / display results

    Args:
        args: Parsed command-line arguments.
    """
    from sar_processor.radar_config import RadarConfig
    from sar_processor.data_loader import load_raw_binary, load_imu_data, load_hdf5, save_hdf5
    from sar_processor.bpa import BackProjectionAlgorithm
    from sar_processor.autofocus import PhaseGradientAutofocus
    from sar_processor.motion_comp import MotionCompensator

    # ---------------------------------------------------------------- #
    # Step 1: Load configuration
    # ---------------------------------------------------------------- #
    if args.config is not None:
        config = RadarConfig.from_file(args.config)
        print(f"Loaded config from: {args.config}")
    else:
        config = RadarConfig.default()
        print("Using default AWR2243 radar configuration.")

    if args.verbose:
        print(config.summary())

    # ---------------------------------------------------------------- #
    # Step 2: Load radar data
    # ---------------------------------------------------------------- #
    input_path = Path(args.input)

    if input_path.suffix in (".h5", ".hdf5"):
        print(f"Loading HDF5 recording: {input_path}")
        recording = load_hdf5(input_path)
        raw_data = recording["radar_data"]
        imu_data = recording.get("imu")
        if recording.get("config") is not None:
            config = recording["config"]
            print("Using configuration from HDF5 file.")
    elif input_path.suffix == ".bin":
        print(f"Loading raw binary: {input_path}")
        raw_data = load_raw_binary(input_path, config)
        # For binary files, squeeze out the frame dimension if single frame
        if raw_data.ndim == 4 and raw_data.shape[0] == 1:
            raw_data = raw_data[0]
        imu_data = None
    else:
        print(f"Error: Unsupported input format: {input_path.suffix}")
        sys.exit(1)

    print(f"Radar data shape: {raw_data.shape}")

    # ---------------------------------------------------------------- #
    # Step 3: Load IMU data (if separate file provided)
    # ---------------------------------------------------------------- #
    if args.imu is not None:
        print(f"Loading IMU data: {args.imu}")
        imu_data = load_imu_data(args.imu)
        print(f"IMU samples: {len(imu_data['timestamps'])}")
    elif imu_data is None:
        print("No IMU data available. Motion compensation will be skipped.")

    # ---------------------------------------------------------------- #
    # Step 4: Range compression
    # ---------------------------------------------------------------- #
    print("\n--- Range Compression ---")
    t0 = time.time()

    bpa = BackProjectionAlgorithm(config, window="hann")

    # Handle data shapes: (num_chirps, num_rx, num_samples) or (num_chirps, num_samples)
    if raw_data.ndim == 3:
        # Multi-channel: range-compress each RX channel
        rc_data = bpa.range_compression(raw_data)
        print(f"Range-compressed data shape: {rc_data.shape}")
    elif raw_data.ndim == 2:
        rc_data = bpa.range_compression(raw_data)
        print(f"Range-compressed data shape: {rc_data.shape}")
    else:
        print(f"Error: Unexpected data shape: {raw_data.shape}")
        sys.exit(1)

    dt_rc = time.time() - t0
    print(f"Range compression: {dt_rc:.3f} s")

    # ---------------------------------------------------------------- #
    # Step 5: Motion compensation
    # ---------------------------------------------------------------- #
    num_pulses = rc_data.shape[0]
    radar_timestamps = np.arange(num_pulses) * config.pri

    if imu_data is not None and not args.no_mocomp:
        print("\n--- Motion Compensation ---")
        t0 = time.time()

        mocomp = MotionCompensator(imu_data, radar_timestamps, config)
        antenna_positions = mocomp.estimate_trajectory()
        rc_data_mocomp = mocomp.compensate(rc_data, antenna_positions)

        dt_mc = time.time() - t0
        print(f"Motion compensation: {dt_mc:.3f} s")
        print(f"Trajectory extent: {antenna_positions.ptp(axis=0)} m")
    else:
        print("\nSkipping motion compensation. Using ideal linear trajectory.")
        antenna_positions = MotionCompensator.generate_ideal_trajectory(
            scan_length=0.5,
            num_pulses=num_pulses,
            scan_axis="x",
            standoff_distance=0.3,
        )
        rc_data_mocomp = rc_data

    # ---------------------------------------------------------------- #
    # Step 6: Back-Projection (image formation)
    # ---------------------------------------------------------------- #
    print("\n--- Back-Projection ---")
    t0 = time.time()

    grid_x, grid_y = bpa.setup_grid(
        x_extent=tuple(args.x_extent),
        y_extent=tuple(args.y_extent),
        pixel_size=args.pixel_size,
    )
    print(f"Image grid: {grid_x.shape[1]} x {grid_x.shape[0]} pixels")

    # Choose GPU or CPU
    use_gpu = args.gpu and not args.cpu

    if use_gpu:
        try:
            from sar_processor.cuda_kernels.bpa_kernel import bpa_cuda, check_cuda_available

            if check_cuda_available():
                print("Using CUDA GPU acceleration.")
                sar_image = bpa_cuda(
                    rc_data_mocomp if rc_data_mocomp.ndim == 2 else np.mean(rc_data_mocomp, axis=1),
                    antenna_positions,
                    grid_x,
                    grid_y,
                    fc=config.fc,
                    range_axis=bpa.range_axis,
                )
            else:
                print("CUDA not available. Falling back to CPU.")
                use_gpu = False
        except ImportError:
            print("CuPy not installed. Falling back to CPU.")
            use_gpu = False

    if not use_gpu:
        print("Using CPU (NumPy) processing.")
        sar_image = bpa.backproject(
            rc_data_mocomp,
            antenna_positions,
            grid_x,
            grid_y,
            verbose=args.verbose,
        )

    dt_bp = time.time() - t0
    print(f"Back-projection: {dt_bp:.3f} s")

    # ---------------------------------------------------------------- #
    # Step 7: Autofocus (PGA)
    # ---------------------------------------------------------------- #
    if not args.no_autofocus:
        print("\n--- Phase Gradient Autofocus ---")
        t0 = time.time()

        pga = PhaseGradientAutofocus(
            max_iter=args.pga_iterations,
            convergence_threshold=0.01,
        )
        sar_image = pga.apply(sar_image)

        dt_af = time.time() - t0
        iterations = len(pga.convergence_history)
        print(f"PGA autofocus: {dt_af:.3f} s ({iterations} iterations)")
        if pga.convergence_history:
            print(f"Final phase error RMS: {pga.convergence_history[-1]:.4f} rad")
    else:
        print("\nSkipping autofocus.")

    # ---------------------------------------------------------------- #
    # Step 8: Save results
    # ---------------------------------------------------------------- #
    if args.output is not None:
        output_path = Path(args.output)
        print(f"\nSaving results to: {output_path}")

        if output_path.suffix in (".h5", ".hdf5"):
            save_hdf5(
                output_path,
                radar_data=raw_data,
                imu_data=imu_data,
                config=config,
                sar_image=sar_image,
                antenna_positions=antenna_positions,
                metadata={
                    "processing_date": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "pixel_size_m": args.pixel_size,
                    "x_extent": str(args.x_extent),
                    "y_extent": str(args.y_extent),
                    "autofocus": str(not args.no_autofocus),
                    "motion_comp": str(not args.no_mocomp),
                },
            )
        elif output_path.suffix == ".npz":
            np.savez_compressed(
                output_path,
                sar_image=sar_image,
                grid_x=grid_x,
                grid_y=grid_y,
                antenna_positions=antenna_positions,
            )
        else:
            print(f"Warning: Unknown output format {output_path.suffix}, saving as .npz")
            np.savez_compressed(
                str(output_path) + ".npz",
                sar_image=sar_image,
                grid_x=grid_x,
                grid_y=grid_y,
            )

        print("Results saved successfully.")

    # ---------------------------------------------------------------- #
    # Step 9: Display
    # ---------------------------------------------------------------- #
    if args.show:
        display_image(sar_image, grid_x, grid_y, args.dynamic_range)

    # Print summary
    print("\n=== Processing Summary ===")
    print(f"  Input:          {args.input}")
    print(f"  Pulses:         {num_pulses}")
    print(f"  Image size:     {sar_image.shape}")
    print(f"  Pixel size:     {args.pixel_size * 1000:.1f} mm")
    print(f"  Range res:      {config.range_resolution * 100:.2f} cm")
    peak_db = 20.0 * np.log10(np.max(np.abs(sar_image)) + 1e-30)
    print(f"  Peak intensity: {peak_db:.1f} dB")


def process_live(args: argparse.Namespace) -> None:
    """Run the live WiFi streaming SAR processing pipeline.

    Connects to the ESP32 on the radar hardware, receives raw data packets
    in real-time, and continuously updates the SAR image.

    Args:
        args: Parsed command-line arguments.
    """
    import socket
    import struct

    from sar_processor.radar_config import RadarConfig
    from sar_processor.bpa import BackProjectionAlgorithm
    from sar_processor.motion_comp import MotionCompensator

    if args.config is not None:
        config = RadarConfig.from_file(args.config)
    else:
        config = RadarConfig.default()

    print(f"Live mode: Connecting to {args.host}:{args.port}")

    bpa = BackProjectionAlgorithm(config)
    grid_x, grid_y = bpa.setup_grid(
        x_extent=tuple(args.x_extent),
        y_extent=tuple(args.y_extent),
        pixel_size=args.pixel_size,
    )

    # Buffers for accumulating data
    rc_buffer = []
    position_buffer = []
    imu_buffer = {"timestamps": [], "acc": [], "gyro": [], "mag": None}

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((args.host, args.port))
        print("Connected to radar. Waiting for data...")

        # Packet header format: [sync(4B), type(1B), length(4B)]
        HEADER_SIZE = 9
        SYNC_WORD = b"\xAA\x55\xAA\x55"

        while True:
            # Read packet header
            header = _recv_exact(sock, HEADER_SIZE)
            if header is None:
                print("Connection lost.")
                break

            sync = header[:4]
            if sync != SYNC_WORD:
                print("Warning: Sync lost, resynchronizing...")
                continue

            pkt_type = header[4]
            pkt_len = struct.unpack("<I", header[5:9])[0]

            # Read packet payload
            payload = _recv_exact(sock, pkt_len)
            if payload is None:
                break

            if pkt_type == 0x01:
                # Radar data packet
                # Parse raw ADC samples for one chirp
                chirp_data = np.frombuffer(payload, dtype=np.int16)
                num_expected = config.num_rx * 2 * config.num_samples
                if len(chirp_data) >= num_expected:
                    chirp_data = chirp_data[:num_expected].reshape(
                        config.num_rx * 2, config.num_samples
                    )
                    # Convert to complex
                    chirp_complex = (
                        chirp_data[0::2].astype(np.float64)
                        + 1j * chirp_data[1::2].astype(np.float64)
                    )
                    # Average RX channels and range-compress
                    chirp_avg = np.mean(chirp_complex, axis=0, keepdims=True)
                    rc = bpa.range_compression(chirp_avg)
                    rc_buffer.append(rc[0])

            elif pkt_type == 0x02:
                # IMU data packet: timestamp(8B) + 6x float(24B) = 32 bytes
                if len(payload) >= 32:
                    ts = struct.unpack("<d", payload[:8])[0]
                    imu_vals = struct.unpack("<6f", payload[8:32])
                    imu_buffer["timestamps"].append(ts)
                    imu_buffer["acc"].append(list(imu_vals[:3]))
                    imu_buffer["gyro"].append(list(imu_vals[3:6]))

            # Periodically form an image
            if len(rc_buffer) > 0 and len(rc_buffer) % 50 == 0:
                print(f"Pulses: {len(rc_buffer)}, forming image...")

                rc_array = np.array(rc_buffer)

                # Estimate positions if we have IMU data
                if len(imu_buffer["timestamps"]) > 10:
                    imu_np = {
                        "timestamps": np.array(imu_buffer["timestamps"]),
                        "acc": np.array(imu_buffer["acc"]),
                        "gyro": np.array(imu_buffer["gyro"]),
                        "mag": None,
                    }
                    radar_ts = np.arange(len(rc_buffer)) * config.pri
                    mc = MotionCompensator(imu_np, radar_ts, config)
                    positions = mc.estimate_trajectory()
                else:
                    positions = MotionCompensator.generate_ideal_trajectory(
                        scan_length=len(rc_buffer) * 0.001,
                        num_pulses=len(rc_buffer),
                    )

                # Form image
                sar_image = bpa.backproject(rc_array, positions, grid_x, grid_y)
                peak_db = 20.0 * np.log10(np.max(np.abs(sar_image)) + 1e-30)
                print(f"  Image formed. Peak: {peak_db:.1f} dB")

    except socket.timeout:
        print("Connection timed out.")
    except ConnectionRefusedError:
        print(f"Connection refused. Is the radar running at {args.host}:{args.port}?")
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        sock.close()
        print(f"Total pulses collected: {len(rc_buffer)}")


def _recv_exact(sock, num_bytes: int) -> Optional[bytes]:
    """Receive exactly num_bytes from a socket.

    Args:
        sock: Connected socket.
        num_bytes: Number of bytes to receive.

    Returns:
        Received bytes, or None if the connection was closed.
    """
    import socket as _socket

    data = b""
    while len(data) < num_bytes:
        try:
            chunk = sock.recv(num_bytes - len(data))
        except _socket.timeout:
            return None
        if not chunk:
            return None
        data += chunk
    return data


def display_image(
    sar_image: np.ndarray,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    dynamic_range: float = 60.0,
) -> None:
    """Display the SAR image using matplotlib.

    Args:
        sar_image: Complex SAR image.
        grid_x: Pixel x-coordinates (meters).
        grid_y: Pixel y-coordinates (meters).
        dynamic_range: Display dynamic range in dB.
    """
    import matplotlib.pyplot as plt

    from sar_processor.bpa import BackProjectionAlgorithm

    db_image = BackProjectionAlgorithm.to_db(sar_image, dynamic_range)

    x_extent = [grid_x.min() * 100, grid_x.max() * 100]  # Convert to cm
    y_extent = [grid_y.min() * 100, grid_y.max() * 100]

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    im = ax.imshow(
        db_image,
        extent=[x_extent[0], x_extent[1], y_extent[1], y_extent[0]],
        aspect="auto",
        cmap="inferno",
        origin="upper",
    )
    ax.set_xlabel("Cross-Range (cm)")
    ax.set_ylabel("Range (cm)")
    ax.set_title("SAR Image (dB)")
    plt.colorbar(im, ax=ax, label="Intensity (dB)")
    plt.tight_layout()
    plt.show()


def main() -> None:
    """Main entry point for the SAR processor CLI."""
    args = parse_args()

    print("=" * 60)
    print("  77 GHz Handheld SAR Radar - Image Processor")
    print("=" * 60)

    # Launch GUI mode
    if args.gui:
        from gui.main_window import run_gui
        run_gui()
        return

    # Live streaming mode
    if args.live:
        process_live(args)
        return

    # Offline file processing mode
    if args.input is None:
        print("Error: --input is required for offline processing.")
        print("Use --live for live mode or --gui for the graphical interface.")
        sys.exit(1)

    process_offline(args)


if __name__ == "__main__":
    main()
