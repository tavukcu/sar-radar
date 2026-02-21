"""
CUDA-Accelerated Back-Projection Algorithm

Provides a GPU-accelerated implementation of the SAR Back-Projection Algorithm
using CuPy raw CUDA kernels. Each GPU thread computes one output pixel,
with shared memory used for antenna positions and texture memory concepts
applied via interpolation for range-compressed data lookup.

Performance target: real-time imaging for a 256x256 grid with ~1000 pulses
on an NVIDIA GPU with CUDA compute capability >= 6.0.

Falls back to a NumPy implementation if CUDA / CuPy is not available.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import cupy as cp
    from cupy import RawKernel

    HAS_CUDA = True
except ImportError:
    HAS_CUDA = False

from sar_processor.radar_config import C


# ---------------------------------------------------------------------- #
# CUDA kernel source code
# ---------------------------------------------------------------------- #

_BPA_KERNEL_SOURCE = r"""
extern "C" __global__
void bpa_kernel(
    const double* rc_real,          // Range-compressed data (real), [num_pulses x num_range_bins]
    const double* rc_imag,          // Range-compressed data (imag), [num_pulses x num_range_bins]
    const double* ant_x,            // Antenna x positions [num_pulses]
    const double* ant_y,            // Antenna y positions [num_pulses]
    const double* ant_z,            // Antenna z positions [num_pulses]
    const double* grid_x,           // Pixel x coordinates [num_pixels]
    const double* grid_y,           // Pixel y coordinates [num_pixels]
    const double* grid_z,           // Pixel z coordinates [num_pixels]
    double* out_real,               // Output image real [num_pixels]
    double* out_imag,               // Output image imag [num_pixels]
    const int num_pulses,
    const int num_range_bins,
    const int num_pixels,
    const double range_start,       // First range bin distance (m)
    const double range_step,        // Range bin spacing (m)
    const double k_c                // Carrier wavenumber: 4*pi*fc/c
)
{
    // Each thread computes one output pixel
    int pid = blockDim.x * blockIdx.x + threadIdx.x;
    if (pid >= num_pixels) return;

    double px = grid_x[pid];
    double py = grid_y[pid];
    double pz = grid_z[pid];

    double sum_real = 0.0;
    double sum_imag = 0.0;

    // Load antenna positions into shared memory for faster access
    extern __shared__ double shared_ant[];
    // shared_ant layout: [x0, y0, z0, x1, y1, z1, ...]
    // Each block cooperatively loads antenna positions
    int tid = threadIdx.x;
    int block_size = blockDim.x;

    for (int i = tid; i < num_pulses; i += block_size) {
        shared_ant[3 * i + 0] = ant_x[i];
        shared_ant[3 * i + 1] = ant_y[i];
        shared_ant[3 * i + 2] = ant_z[i];
    }
    __syncthreads();

    // Back-projection loop over all pulses
    for (int n = 0; n < num_pulses; n++) {
        double ax = shared_ant[3 * n + 0];
        double ay = shared_ant[3 * n + 1];
        double az = shared_ant[3 * n + 2];

        // Compute distance from antenna to pixel
        double dx = px - ax;
        double dy = py - ay;
        double dz = pz - az;
        double dist = sqrt(dx * dx + dy * dy + dz * dz);

        // Convert distance to fractional range bin index
        double frac_idx = (dist - range_start) / range_step;

        // Bounds check
        if (frac_idx < 0.0 || frac_idx >= (double)(num_range_bins - 1)) continue;

        // Linear interpolation of range-compressed data
        int idx_low = (int)frac_idx;
        int idx_high = idx_low + 1;
        if (idx_high >= num_range_bins) idx_high = num_range_bins - 1;
        double frac = frac_idx - (double)idx_low;

        int base_low = n * num_range_bins + idx_low;
        int base_high = n * num_range_bins + idx_high;

        double rc_r = rc_real[base_low] * (1.0 - frac) + rc_real[base_high] * frac;
        double rc_i = rc_imag[base_low] * (1.0 - frac) + rc_imag[base_high] * frac;

        // Phase compensation: exp(j * k_c * dist)
        double phase = k_c * dist;
        double cos_ph = cos(phase);
        double sin_ph = sin(phase);

        // Complex multiply: (rc_r + j*rc_i) * (cos_ph + j*sin_ph)
        sum_real += rc_r * cos_ph - rc_i * sin_ph;
        sum_imag += rc_r * sin_ph + rc_i * cos_ph;
    }

    out_real[pid] = sum_real;
    out_imag[pid] = sum_imag;
}
"""

# Compile the kernel once (lazy initialization)
_compiled_kernel: Optional[object] = None


def _get_kernel() -> object:
    """Get or compile the CUDA BPA kernel."""
    global _compiled_kernel
    if _compiled_kernel is None:
        _compiled_kernel = RawKernel(_BPA_KERNEL_SOURCE, "bpa_kernel")
    return _compiled_kernel


def bpa_cuda(
    range_compressed: np.ndarray,
    antenna_positions: np.ndarray,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    grid_z: Optional[np.ndarray] = None,
    wavelength: float = C / 78.5e9,
    fc: float = 78.5e9,
    range_axis: Optional[np.ndarray] = None,
    block_size: int = 256,
) -> np.ndarray:
    """GPU-accelerated back-projection using CUDA.

    Each GPU thread computes one output pixel by looping over all pulses,
    computing the range to the pixel, interpolating the range-compressed
    data, applying phase compensation, and accumulating the result.

    Antenna positions are loaded into shared memory for each thread block
    to reduce global memory bandwidth.

    Args:
        range_compressed: Range-compressed data, shape (num_pulses, num_range_bins).
            Complex numpy array.
        antenna_positions: Antenna phase center positions, shape (num_pulses, 3).
        grid_x: Pixel x-coordinates (flattened or any shape).
        grid_y: Pixel y-coordinates (same shape as grid_x).
        grid_z: Optional pixel z-coordinates. Defaults to zeros.
        wavelength: Radar wavelength in meters (used if fc not provided).
        fc: Center frequency in Hz. Default 78.5 GHz.
        range_axis: Range axis in meters for the compressed data. If None,
            computed from the data dimensions assuming standard parameters.
        block_size: CUDA thread block size. Should be a multiple of 32 (warp size).

    Returns:
        Complex SAR image as a numpy array with the same shape as grid_x.

    Raises:
        ImportError: If CuPy is not available (falls back to CPU with warning).
    """
    if not HAS_CUDA:
        print("Warning: CUDA not available, falling back to CPU implementation.")
        return bpa_cpu(
            range_compressed, antenna_positions,
            grid_x, grid_y, grid_z, fc, range_axis,
        )

    output_shape = grid_x.shape
    num_pulses, num_range_bins = range_compressed.shape
    num_pixels = grid_x.size

    # Flatten grid arrays
    gx_flat = grid_x.ravel().astype(np.float64)
    gy_flat = grid_y.ravel().astype(np.float64)
    if grid_z is not None:
        gz_flat = grid_z.ravel().astype(np.float64)
    else:
        gz_flat = np.zeros(num_pixels, dtype=np.float64)

    # Compute range axis if not provided
    if range_axis is None:
        # Assume default config: 5 GHz BW, 60 us chirp, 10 MHz ADC
        slope = 5e9 / 60e-6
        fs = 10e6
        freq_axis = np.fft.fftfreq(num_range_bins * 2, d=1.0 / fs)[:num_range_bins]
        range_axis = freq_axis * C / (2.0 * slope)

    range_start = float(range_axis[0])
    range_step = float(range_axis[1] - range_axis[0]) if len(range_axis) > 1 else 1.0

    # Carrier wavenumber
    k_c = 4.0 * np.pi * fc / C

    # Transfer data to GPU
    rc_real_gpu = cp.asarray(np.real(range_compressed).astype(np.float64).ravel())
    rc_imag_gpu = cp.asarray(np.imag(range_compressed).astype(np.float64).ravel())
    ant_x_gpu = cp.asarray(antenna_positions[:, 0].astype(np.float64))
    ant_y_gpu = cp.asarray(antenna_positions[:, 1].astype(np.float64))
    ant_z_gpu = cp.asarray(antenna_positions[:, 2].astype(np.float64))
    gx_gpu = cp.asarray(gx_flat)
    gy_gpu = cp.asarray(gy_flat)
    gz_gpu = cp.asarray(gz_flat)

    # Output arrays on GPU
    out_real_gpu = cp.zeros(num_pixels, dtype=cp.float64)
    out_imag_gpu = cp.zeros(num_pixels, dtype=cp.float64)

    # Launch kernel
    kernel = _get_kernel()
    grid_size = (num_pixels + block_size - 1) // block_size
    shared_mem_size = num_pulses * 3 * 8  # 3 doubles per pulse

    # Cap shared memory; if too large, fall back to non-shared version
    max_shared = 48 * 1024  # 48 KB typical limit
    if shared_mem_size > max_shared:
        # For very large pulse counts, process in chunks
        return _bpa_cuda_chunked(
            range_compressed, antenna_positions,
            grid_x, grid_y, grid_z, fc, range_axis, block_size,
        )

    kernel(
        (grid_size,),
        (block_size,),
        (
            rc_real_gpu, rc_imag_gpu,
            ant_x_gpu, ant_y_gpu, ant_z_gpu,
            gx_gpu, gy_gpu, gz_gpu,
            out_real_gpu, out_imag_gpu,
            np.int32(num_pulses),
            np.int32(num_range_bins),
            np.int32(num_pixels),
            np.float64(range_start),
            np.float64(range_step),
            np.float64(k_c),
        ),
        shared_mem=shared_mem_size,
    )

    # Transfer result back to CPU
    out_real = cp.asnumpy(out_real_gpu)
    out_imag = cp.asnumpy(out_imag_gpu)

    image = (out_real + 1j * out_imag).reshape(output_shape)
    return image


def _bpa_cuda_chunked(
    range_compressed: np.ndarray,
    antenna_positions: np.ndarray,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    grid_z: Optional[np.ndarray],
    fc: float,
    range_axis: np.ndarray,
    block_size: int,
    chunk_size: int = 1000,
) -> np.ndarray:
    """Process BPA in chunks when pulse count exceeds shared memory limits.

    Splits the pulses into chunks that fit in shared memory, processes each
    chunk, and accumulates the results.

    Args:
        range_compressed: Range-compressed data (num_pulses, num_range_bins).
        antenna_positions: Antenna positions (num_pulses, 3).
        grid_x, grid_y, grid_z: Pixel coordinates.
        fc: Center frequency.
        range_axis: Range axis array.
        block_size: CUDA block size.
        chunk_size: Number of pulses per chunk.

    Returns:
        Complex SAR image.
    """
    num_pulses = range_compressed.shape[0]
    output_shape = grid_x.shape
    image = np.zeros(grid_x.size, dtype=np.complex128)

    for start in range(0, num_pulses, chunk_size):
        end = min(start + chunk_size, num_pulses)
        chunk_image = bpa_cuda(
            range_compressed[start:end],
            antenna_positions[start:end],
            grid_x,
            grid_y,
            grid_z,
            fc=fc,
            range_axis=range_axis,
            block_size=block_size,
        )
        image += chunk_image.ravel()

    return image.reshape(output_shape)


def bpa_cpu(
    range_compressed: np.ndarray,
    antenna_positions: np.ndarray,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    grid_z: Optional[np.ndarray] = None,
    fc: float = 78.5e9,
    range_axis: Optional[np.ndarray] = None,
) -> np.ndarray:
    """CPU fallback implementation of back-projection using NumPy.

    This is the pure NumPy implementation used when CUDA is not available.
    It is functionally identical to the GPU version but significantly slower
    for large grids.

    Args:
        range_compressed: Range-compressed data (num_pulses, num_range_bins).
        antenna_positions: Antenna positions (num_pulses, 3).
        grid_x: Pixel x-coordinates.
        grid_y: Pixel y-coordinates.
        grid_z: Optional pixel z-coordinates.
        fc: Center frequency in Hz.
        range_axis: Range axis in meters.

    Returns:
        Complex SAR image with the same shape as grid_x.
    """
    output_shape = grid_x.shape
    num_pulses, num_range_bins = range_compressed.shape
    num_pixels = grid_x.size

    gx = grid_x.ravel()
    gy = grid_y.ravel()
    gz = grid_z.ravel() if grid_z is not None else np.zeros(num_pixels)

    # Range axis
    if range_axis is None:
        slope = 5e9 / 60e-6
        fs = 10e6
        freq_axis = np.fft.fftfreq(num_range_bins * 2, d=1.0 / fs)[:num_range_bins]
        range_axis = freq_axis * C / (2.0 * slope)

    range_start = range_axis[0]
    range_step = range_axis[1] - range_axis[0] if len(range_axis) > 1 else 1.0

    k_c = 4.0 * np.pi * fc / C

    image = np.zeros(num_pixels, dtype=np.complex128)

    for n in range(num_pulses):
        ax, ay, az = antenna_positions[n]

        dx = gx - ax
        dy = gy - ay
        dz = gz - az
        distances = np.sqrt(dx**2 + dy**2 + dz**2)

        # Fractional range bin index
        frac_idx = (distances - range_start) / range_step
        valid = (frac_idx >= 0) & (frac_idx < num_range_bins - 1)

        idx_low = np.clip(frac_idx.astype(np.int32), 0, num_range_bins - 2)
        idx_high = idx_low + 1
        frac = frac_idx - idx_low

        # Linear interpolation
        rc_interp = (
            range_compressed[n, idx_low] * (1.0 - frac)
            + range_compressed[n, idx_high] * frac
        )

        # Phase compensation
        phase_comp = np.exp(1j * k_c * distances)

        # Accumulate with validity mask
        image += np.where(valid, rc_interp * phase_comp, 0.0)

    return image.reshape(output_shape)


def check_cuda_available() -> bool:
    """Check if CUDA acceleration is available.

    Returns:
        True if CuPy is installed and a CUDA GPU is detected.
    """
    if not HAS_CUDA:
        return False
    try:
        cp.cuda.Device(0).compute_capability
        return True
    except cp.cuda.runtime.CUDARuntimeError:
        return False


def get_gpu_info() -> dict:
    """Get information about the available CUDA GPU.

    Returns:
        Dictionary with GPU name, memory, compute capability, etc.
        Empty dict if CUDA is not available.
    """
    if not check_cuda_available():
        return {}

    device = cp.cuda.Device(0)
    mem_info = device.mem_info
    return {
        "name": cp.cuda.runtime.getDeviceProperties(0)["name"].decode(),
        "compute_capability": device.compute_capability,
        "total_memory_mb": mem_info[1] / (1024 * 1024),
        "free_memory_mb": mem_info[0] / (1024 * 1024),
    }
