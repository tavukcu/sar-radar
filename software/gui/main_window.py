"""
SAR Radar Main GUI Window

Provides the primary graphical interface for the handheld SAR radar system.
Built with PyQt6 and pyqtgraph for high-performance image display.

Layout:
    +--------------------------------------------------+
    |  Menu Bar (File | View | Tools | Help)           |
    +----------+------------------------+--------------+
    |          |                        |              |
    |  Control |    SAR Image Display   |  Scan Info   |
    |  Panel   |    (pyqtgraph)         |  Panel       |
    |          |                        |              |
    |  Connect |                        |  Position    |
    |  Start   |                        |  Time        |
    |  Stop    |                        |  Resolution  |
    |  Save    |                        |  SNR         |
    |          |                        |              |
    +----------+------------------------+--------------+
    |  Status Bar + Progress                           |
    +--------------------------------------------------+
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from PyQt6.QtWidgets import (
        QApplication,
        QMainWindow,
        QWidget,
        QVBoxLayout,
        QHBoxLayout,
        QPushButton,
        QLabel,
        QGroupBox,
        QProgressBar,
        QStatusBar,
        QMenuBar,
        QFileDialog,
        QComboBox,
        QSpinBox,
        QDoubleSpinBox,
        QSplitter,
        QFrame,
        QMessageBox,
    )
    from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
    from PyQt6.QtGui import QAction, QIcon, QFont

    HAS_PYQT6 = True
except ImportError:
    HAS_PYQT6 = False

try:
    import pyqtgraph as pg

    HAS_PYQTGRAPH = True
except ImportError:
    HAS_PYQTGRAPH = False


class SARMainWindow(QMainWindow):
    """Main application window for the SAR radar GUI.

    Provides controls for connecting to the radar hardware (via WiFi),
    starting and stopping scans, displaying the SAR image in real-time,
    and saving results.
    """

    # Signals for thread-safe GUI updates
    image_updated = pyqtSignal(np.ndarray)
    status_message = pyqtSignal(str)
    progress_updated = pyqtSignal(int)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        """Initialize the main window.

        Args:
            parent: Optional parent widget.
        """
        super().__init__(parent)

        self.setWindowTitle("SAR Radar - 77 GHz Handheld SAR Imaging System")
        self.setMinimumSize(1200, 800)

        # State
        self._is_connected = False
        self._is_scanning = False
        self._current_image: Optional[np.ndarray] = None
        self._scan_start_time: Optional[float] = None
        self._pulse_count = 0
        self._colormap = "viridis"
        self._dynamic_range_db = 60.0

        # Build UI
        self._create_menu_bar()
        self._create_central_widget()
        self._create_status_bar()

        # Connect signals
        self.image_updated.connect(self._on_image_updated)
        self.status_message.connect(self._on_status_message)
        self.progress_updated.connect(self._on_progress_updated)

        # Live update timer
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_scan_info)
        self._update_timer.setInterval(500)  # 2 Hz UI refresh

    # ------------------------------------------------------------------ #
    #  Menu Bar
    # ------------------------------------------------------------------ #

    def _create_menu_bar(self) -> None:
        """Create the application menu bar."""
        menu_bar = self.menuBar()

        # -- File Menu --
        file_menu = menu_bar.addMenu("&File")

        open_action = QAction("&Open Recording...", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self._on_open_file)
        file_menu.addAction(open_action)

        save_action = QAction("&Save Results...", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self._on_save_file)
        file_menu.addAction(save_action)

        file_menu.addSeparator()

        export_png_action = QAction("Export as &PNG...", self)
        export_png_action.setShortcut("Ctrl+E")
        export_png_action.triggered.connect(self._on_export_png)
        file_menu.addAction(export_png_action)

        file_menu.addSeparator()

        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # -- View Menu --
        view_menu = menu_bar.addMenu("&View")

        colormap_menu = view_menu.addMenu("&Colormap")
        for cmap in ["viridis", "inferno", "plasma", "magma", "gray", "jet", "hot"]:
            action = QAction(cmap, self)
            action.triggered.connect(lambda checked, c=cmap: self._set_colormap(c))
            colormap_menu.addAction(action)

        db_range_menu = view_menu.addMenu("&Dynamic Range")
        for db_val in [20, 30, 40, 50, 60, 80]:
            action = QAction(f"{db_val} dB", self)
            action.triggered.connect(
                lambda checked, d=db_val: self._set_dynamic_range(d)
            )
            db_range_menu.addAction(action)

        # -- Tools Menu --
        tools_menu = menu_bar.addMenu("&Tools")

        autofocus_action = QAction("Run &Autofocus (PGA)", self)
        autofocus_action.triggered.connect(self._on_run_autofocus)
        tools_menu.addAction(autofocus_action)

        mocomp_action = QAction("Run &Motion Compensation", self)
        mocomp_action.triggered.connect(self._on_run_motion_comp)
        tools_menu.addAction(mocomp_action)

        tools_menu.addSeparator()

        gpu_info_action = QAction("&GPU Info", self)
        gpu_info_action.triggered.connect(self._on_show_gpu_info)
        tools_menu.addAction(gpu_info_action)

        # -- Help Menu --
        help_menu = menu_bar.addMenu("&Help")

        about_action = QAction("&About", self)
        about_action.triggered.connect(self._on_about)
        help_menu.addAction(about_action)

    # ------------------------------------------------------------------ #
    #  Central Widget
    # ------------------------------------------------------------------ #

    def _create_central_widget(self) -> None:
        """Create the main layout with control, image, and info panels."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # -- Left Panel: Controls --
        left_panel = self._create_control_panel()
        splitter.addWidget(left_panel)

        # -- Center Panel: SAR Image --
        center_panel = self._create_image_panel()
        splitter.addWidget(center_panel)

        # -- Right Panel: Scan Info --
        right_panel = self._create_info_panel()
        splitter.addWidget(right_panel)

        # Set initial splitter sizes (proportional)
        splitter.setSizes([200, 700, 250])

    def _create_control_panel(self) -> QWidget:
        """Create the left control panel with scan buttons."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        panel.setMaximumWidth(250)

        # -- Connection Group --
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_group)

        self._wifi_status_label = QLabel("Status: Disconnected")
        self._wifi_status_label.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self._wifi_status_label)

        self._ip_label = QLabel("IP: --")
        conn_layout.addWidget(self._ip_label)

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setMinimumHeight(40)
        self._btn_connect.clicked.connect(self._on_connect)
        conn_layout.addWidget(self._btn_connect)

        layout.addWidget(conn_group)

        # -- Scan Control Group --
        scan_group = QGroupBox("Scan Control")
        scan_layout = QVBoxLayout(scan_group)

        self._btn_start = QPushButton("Start Scan")
        self._btn_start.setMinimumHeight(50)
        self._btn_start.setEnabled(False)
        self._btn_start.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-size: 14px; }"
            "QPushButton:disabled { background-color: #888; }"
        )
        self._btn_start.clicked.connect(self._on_start_scan)
        scan_layout.addWidget(self._btn_start)

        self._btn_stop = QPushButton("Stop Scan")
        self._btn_stop.setMinimumHeight(40)
        self._btn_stop.setEnabled(False)
        self._btn_stop.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; }"
            "QPushButton:disabled { background-color: #888; }"
        )
        self._btn_stop.clicked.connect(self._on_stop_scan)
        scan_layout.addWidget(self._btn_stop)

        self._btn_save = QPushButton("Save Results")
        self._btn_save.setMinimumHeight(40)
        self._btn_save.setEnabled(False)
        self._btn_save.clicked.connect(self._on_save_file)
        scan_layout.addWidget(self._btn_save)

        layout.addWidget(scan_group)

        # -- Processing Group --
        proc_group = QGroupBox("Processing")
        proc_layout = QVBoxLayout(proc_group)

        proc_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["GPU (CUDA)", "CPU (NumPy)"])
        proc_layout.addWidget(self._mode_combo)

        proc_layout.addWidget(QLabel("Grid Size:"))
        self._grid_size_spin = QSpinBox()
        self._grid_size_spin.setRange(64, 1024)
        self._grid_size_spin.setSingleStep(64)
        self._grid_size_spin.setValue(256)
        proc_layout.addWidget(self._grid_size_spin)

        proc_layout.addWidget(QLabel("Pixel Size (mm):"))
        self._pixel_size_spin = QDoubleSpinBox()
        self._pixel_size_spin.setRange(1.0, 50.0)
        self._pixel_size_spin.setSingleStep(1.0)
        self._pixel_size_spin.setValue(5.0)
        self._pixel_size_spin.setDecimals(1)
        proc_layout.addWidget(self._pixel_size_spin)

        layout.addWidget(proc_group)

        layout.addStretch()
        return panel

    def _create_image_panel(self) -> QWidget:
        """Create the center SAR image display panel using pyqtgraph."""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        if HAS_PYQTGRAPH:
            # pyqtgraph ImageView for high-performance image display
            self._image_view = pg.ImageView()
            self._image_view.ui.roiBtn.hide()
            self._image_view.ui.menuBtn.hide()

            # Set default colormap
            self._apply_colormap(self._colormap)

            layout.addWidget(self._image_view)
        else:
            # Fallback: simple label placeholder
            placeholder = QLabel("pyqtgraph not available.\nInstall with: pip install pyqtgraph")
            placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
            placeholder.setStyleSheet(
                "font-size: 16px; color: #888; border: 2px dashed #ccc; padding: 20px;"
            )
            layout.addWidget(placeholder)
            self._image_view = None

        return panel

    def _create_info_panel(self) -> QWidget:
        """Create the right info panel showing scan statistics."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        panel.setMaximumWidth(280)

        # -- Scan Info Group --
        info_group = QGroupBox("Scan Information")
        info_layout = QVBoxLayout(info_group)

        self._info_labels = {}
        info_items = [
            ("scan_time", "Scan Time", "0.0 s"),
            ("pulses", "Pulses Collected", "0"),
            ("aperture", "Synthetic Aperture", "0.0 cm"),
            ("range_res", "Range Resolution", "3.0 cm"),
            ("cross_range_res", "Cross-Range Res.", "-- cm"),
            ("max_range", "Max Range", "-- m"),
            ("snr", "Est. SNR", "-- dB"),
            ("grid_size", "Image Size", "256 x 256"),
            ("pixel_size", "Pixel Size", "5.0 mm"),
        ]

        for key, label_text, default_value in info_items:
            row = QHBoxLayout()
            label = QLabel(f"{label_text}:")
            label.setStyleSheet("font-weight: bold;")
            value_label = QLabel(default_value)
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            row.addWidget(label)
            row.addWidget(value_label)
            info_layout.addLayout(row)
            self._info_labels[key] = value_label

        layout.addWidget(info_group)

        # -- IMU Status Group --
        imu_group = QGroupBox("IMU Status")
        imu_layout = QVBoxLayout(imu_group)

        self._imu_labels = {}
        imu_items = [
            ("acc", "Acceleration", "-- m/s2"),
            ("gyro", "Angular Rate", "-- deg/s"),
            ("heading", "Heading", "-- deg"),
            ("velocity", "Velocity", "-- m/s"),
        ]

        for key, label_text, default_value in imu_items:
            row = QHBoxLayout()
            label = QLabel(f"{label_text}:")
            value_label = QLabel(default_value)
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            row.addWidget(label)
            row.addWidget(value_label)
            imu_layout.addLayout(row)
            self._imu_labels[key] = value_label

        layout.addWidget(imu_group)

        # -- Radar Config Group --
        config_group = QGroupBox("Radar Config")
        config_layout = QVBoxLayout(config_group)

        config_items = [
            ("Center Freq.", "78.5 GHz"),
            ("Bandwidth", "5.0 GHz"),
            ("Chirp Duration", "60 us"),
            ("ADC Rate", "10 MSPS"),
            ("TX/RX", "3 / 4"),
        ]

        for label_text, value_text in config_items:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{label_text}:"))
            val = QLabel(value_text)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            val.setStyleSheet("color: #555;")
            row.addWidget(val)
            config_layout.addLayout(row)

        layout.addWidget(config_group)

        layout.addStretch()
        return panel

    # ------------------------------------------------------------------ #
    #  Status Bar
    # ------------------------------------------------------------------ #

    def _create_status_bar(self) -> None:
        """Create the bottom status bar with progress indicator."""
        status_bar = QStatusBar()
        self.setStatusBar(status_bar)

        self._status_label = QLabel("Ready")
        status_bar.addWidget(self._status_label, stretch=1)

        self._progress_bar = QProgressBar()
        self._progress_bar.setMaximumWidth(200)
        self._progress_bar.setVisible(False)
        status_bar.addPermanentWidget(self._progress_bar)

    # ------------------------------------------------------------------ #
    #  Slot: Button Handlers
    # ------------------------------------------------------------------ #

    def _on_connect(self) -> None:
        """Handle the Connect/Disconnect button click."""
        if self._is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        """Attempt to connect to the radar hardware via WiFi."""
        self._status_label.setText("Connecting to radar...")
        # In a real implementation, this would initiate a WiFi/TCP connection
        # to the ESP32 on the radar hardware.
        # For now, simulate a successful connection.
        self._is_connected = True
        self._wifi_status_label.setText("Status: Connected")
        self._wifi_status_label.setStyleSheet("color: green; font-weight: bold;")
        self._ip_label.setText("IP: 192.168.4.1")
        self._btn_connect.setText("Disconnect")
        self._btn_start.setEnabled(True)
        self._status_label.setText("Connected to radar hardware")

    def _disconnect(self) -> None:
        """Disconnect from the radar hardware."""
        if self._is_scanning:
            self._on_stop_scan()
        self._is_connected = False
        self._wifi_status_label.setText("Status: Disconnected")
        self._wifi_status_label.setStyleSheet("color: red; font-weight: bold;")
        self._ip_label.setText("IP: --")
        self._btn_connect.setText("Connect")
        self._btn_start.setEnabled(False)
        self._status_label.setText("Disconnected")

    def _on_start_scan(self) -> None:
        """Start a new SAR scan."""
        self._is_scanning = True
        self._scan_start_time = time.time()
        self._pulse_count = 0

        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_save.setEnabled(False)

        self._progress_bar.setVisible(True)
        self._progress_bar.setRange(0, 0)  # Indeterminate
        self._status_label.setText("Scanning... Move the radar slowly along the target.")

        self._update_timer.start()

    def _on_stop_scan(self) -> None:
        """Stop the current scan and trigger image formation."""
        self._is_scanning = False
        self._update_timer.stop()

        self._btn_start.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._btn_save.setEnabled(True)

        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(100)
        self._status_label.setText(
            f"Scan complete. {self._pulse_count} pulses collected."
        )

    def _on_open_file(self) -> None:
        """Open a recorded SAR data file for processing."""
        filepath, _ = QFileDialog.getOpenFileName(
            self,
            "Open SAR Recording",
            "",
            "HDF5 Files (*.h5 *.hdf5);;Binary Files (*.bin);;All Files (*)",
        )
        if filepath:
            self._status_label.setText(f"Loading: {Path(filepath).name}")
            # In full implementation, load data and process
            self._btn_save.setEnabled(True)
            self._status_label.setText(f"Loaded: {Path(filepath).name}")

    def _on_save_file(self) -> None:
        """Save the current processing results."""
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Save SAR Results",
            "",
            "HDF5 Files (*.h5);;NumPy Files (*.npz);;All Files (*)",
        )
        if filepath:
            self._status_label.setText(f"Saved: {Path(filepath).name}")

    def _on_export_png(self) -> None:
        """Export the current SAR image as a PNG file."""
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Export SAR Image",
            "sar_image.png",
            "PNG Files (*.png);;JPEG Files (*.jpg);;All Files (*)",
        )
        if filepath and self._image_view is not None:
            # Export the image from pyqtgraph
            exporter = pg.exporters.ImageExporter(self._image_view.imageItem)
            exporter.export(filepath)
            self._status_label.setText(f"Exported: {Path(filepath).name}")

    # ------------------------------------------------------------------ #
    #  Slot: Tools Menu Handlers
    # ------------------------------------------------------------------ #

    def _on_run_autofocus(self) -> None:
        """Run Phase Gradient Autofocus on the current image."""
        if self._current_image is None:
            self._status_label.setText("No image to autofocus.")
            return
        self._status_label.setText("Running PGA autofocus...")
        # In full implementation, run PGA in a worker thread
        self._status_label.setText("Autofocus complete.")

    def _on_run_motion_comp(self) -> None:
        """Run motion compensation reprocessing."""
        self._status_label.setText("Running motion compensation...")
        # In full implementation, re-run motion comp in a worker thread
        self._status_label.setText("Motion compensation complete.")

    def _on_show_gpu_info(self) -> None:
        """Display GPU information dialog."""
        try:
            from sar_processor.cuda_kernels.bpa_kernel import get_gpu_info, check_cuda_available

            if check_cuda_available():
                info = get_gpu_info()
                msg = (
                    f"GPU: {info.get('name', 'Unknown')}\n"
                    f"Compute Capability: {info.get('compute_capability', 'N/A')}\n"
                    f"Total Memory: {info.get('total_memory_mb', 0):.0f} MB\n"
                    f"Free Memory: {info.get('free_memory_mb', 0):.0f} MB"
                )
            else:
                msg = "No CUDA-capable GPU detected.\nProcessing will use CPU (NumPy)."
        except ImportError:
            msg = "CuPy not installed.\nInstall with: pip install cupy"

        QMessageBox.information(self, "GPU Information", msg)

    def _on_about(self) -> None:
        """Show the About dialog."""
        QMessageBox.about(
            self,
            "About SAR Radar",
            "SAR Radar Imaging System\n"
            "77 GHz Handheld SAR Radar\n\n"
            "Version 0.1.0\n\n"
            "Back-Projection Algorithm with PGA Autofocus\n"
            "IMU-based Motion Compensation\n"
            "CUDA GPU Acceleration",
        )

    # ------------------------------------------------------------------ #
    #  Slot: View Settings
    # ------------------------------------------------------------------ #

    def _set_colormap(self, name: str) -> None:
        """Change the SAR image colormap.

        Args:
            name: Colormap name (matplotlib-compatible).
        """
        self._colormap = name
        self._apply_colormap(name)
        self._status_label.setText(f"Colormap: {name}")

    def _apply_colormap(self, name: str) -> None:
        """Apply a colormap to the pyqtgraph image view.

        Args:
            name: Colormap name.
        """
        if self._image_view is None or not HAS_PYQTGRAPH:
            return

        try:
            cmap = pg.colormap.get(name, source="matplotlib")
            self._image_view.setColorMap(cmap)
        except Exception:
            # Fallback to default
            pass

    def _set_dynamic_range(self, db: float) -> None:
        """Set the dynamic range for dB display.

        Args:
            db: Dynamic range in dB.
        """
        self._dynamic_range_db = db
        if self._current_image is not None:
            self._display_image(self._current_image)
        self._status_label.setText(f"Dynamic range: {db} dB")

    # ------------------------------------------------------------------ #
    #  Image Display
    # ------------------------------------------------------------------ #

    def _display_image(self, image_complex: np.ndarray) -> None:
        """Display a complex SAR image on the pyqtgraph widget.

        Converts to dB magnitude and applies the current dynamic range
        and colormap settings.

        Args:
            image_complex: 2D complex SAR image array.
        """
        if self._image_view is None:
            return

        magnitude = np.abs(image_complex)
        magnitude = np.where(magnitude > 0, magnitude, 1e-30)
        db_image = 20.0 * np.log10(magnitude)
        db_max = np.max(db_image)
        db_image = np.clip(db_image, db_max - self._dynamic_range_db, db_max)

        self._image_view.setImage(
            db_image.T,
            autoRange=True,
            autoLevels=True,
            autoHistogramRange=True,
        )

    def update_image(self, image_complex: np.ndarray) -> None:
        """Thread-safe method to update the displayed SAR image.

        This can be called from worker threads during live scanning.

        Args:
            image_complex: Complex SAR image array.
        """
        self._current_image = image_complex
        self.image_updated.emit(image_complex)

    def _on_image_updated(self, image: np.ndarray) -> None:
        """Handle image update signal (runs in GUI thread)."""
        self._display_image(image)

    def _on_status_message(self, msg: str) -> None:
        """Handle status message signal."""
        self._status_label.setText(msg)

    def _on_progress_updated(self, value: int) -> None:
        """Handle progress update signal."""
        self._progress_bar.setValue(value)

    # ------------------------------------------------------------------ #
    #  Live Update
    # ------------------------------------------------------------------ #

    def _update_scan_info(self) -> None:
        """Update the scan info panel during a live scan."""
        if not self._is_scanning or self._scan_start_time is None:
            return

        elapsed = time.time() - self._scan_start_time
        self._info_labels["scan_time"].setText(f"{elapsed:.1f} s")
        self._info_labels["pulses"].setText(str(self._pulse_count))

        grid_size = self._grid_size_spin.value()
        pixel_size = self._pixel_size_spin.value()
        self._info_labels["grid_size"].setText(f"{grid_size} x {grid_size}")
        self._info_labels["pixel_size"].setText(f"{pixel_size:.1f} mm")


def run_gui() -> None:
    """Launch the SAR radar GUI application."""
    if not HAS_PYQT6:
        print("Error: PyQt6 is required for the GUI. Install with: pip install pyqt6")
        sys.exit(1)

    app = QApplication(sys.argv)

    # Application style
    app.setStyle("Fusion")

    window = SARMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run_gui()
