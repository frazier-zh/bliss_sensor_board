"""
    Project: BLISS Sensor Test Board (Dual Channel)
    Application: LMP91000 User Interface
    
"""
# --- CONSTANTS ---
# * DO NOT CHANGE *
ADC_SAMPLING_FREQ = 20
BUFFER_LENGTH = 900  # (s)
ADC_RESOLUTION = 10
ADC_N = (1 << ADC_RESOLUTION) - 1
LMP91000_VREF = 3.0
ADC_VREF = 3.3
ADC_VSTEP = ADC_VREF / ADC_N
ADC_YMIN = -100
ADC_YMAX = ADC_N + 100
REFRESH_RATE = 60

# --- Python packages ---
import sys
import serial
import serial.tools.list_ports
import re
import math
import time
import csv
import numpy
from dataclasses import dataclass
from functools import partial

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QComboBox, QTextEdit, QGridLayout,
    QGroupBox, QFileDialog
)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
import pyqtgraph as pg

from lib import deque

# --- Serial worker ---
class LMP91000SerialWorker(QThread):
    """
    Worker thread for serial I/O and simulation. Runs independently and emits signals
    to safely update the UI without blocking the main thread.
    """
    # Signals: emit data and status updates safely to UI
    # Outbound signals
    settings_changed = pyqtSignal()           # register settings changed
    log_message = pyqtSignal(str)             # log string

    # Inbound signals
    ui_settings_changed = pyqtSignal(int, str, int)
    
    def __init__(self, adc_sampling_freq, buffer_length):
        super().__init__()
        self.adc_sampling_freq = adc_sampling_freq
        self.adc_dt = 1.0 / self.adc_sampling_freq
        self.adc_dt_half = self.adc_dt / 2
        self.buffer_length = buffer_length

        self.buffer = deque(self.buffer_length * self.adc_sampling_freq, n_channels=2)
        self.active_buffer = deque(self.buffer_length * self.adc_sampling_freq, n_channels=2, dtype=bool)

        self.current_time = 0
        self.simulate = False
        self.serial_port = None
        self.running = False

        self.switch = False
        self.last_switch_time = 0

        self.c1_settings = LMP91000Settings(chip=1)
        self.c2_settings = LMP91000Settings(chip=2)

        # Connect inbound signals to methods
        self.ui_settings_changed.connect(self.on_ui_settings_changed)

    def on_ui_settings_changed(self, chip, key, value):
        settings = self.c1_settings if chip == 1 else self.c2_settings
        data = settings.update(key, value)
        self.write_serial(data)
        
    def run(self):
        """Main thread loop: read serial or simulate data at sampling frequency."""
        start_time = time.time()
        while True:
            if not self.running:
                time.sleep(0.01)
                continue
            
            self.current_time = time.time() - start_time
            if self.switch:
                self.switch_channel()
            if self.simulate:
                self.simulate_data()
                time.sleep(self.adc_dt)
            else:
                self.read_serial()
                time.sleep(self.adc_dt_half)

    def write_serial(self, data: str):
        if not self.serial_port:
            self.log_message.emit("Not connected.")
            return
        self.serial_port.write(data.encode())
        for line in data.splitlines():
            self.log_message.emit(f"Sent: {line}")

    def read_serial(self):
        """Read and parse a single line from serial port."""
        try:
            line = self.serial_port.readline().decode(errors="ignore").strip()
            if not line:
                return
            
            match = re.search(r"ADC (\d+) (\d+)", line)
            if match:
                v1 = int(match.group(1))
                v2 = int(match.group(2))
                self.buffer.append([v1, v2], self.current_time)
                self.active_buffer.append([self.c1_settings.active, self.c2_settings.active], self.current_time)
            else:
                # non-ADC lines go to log
                self.log_message.emit(line)
        except Exception as e:
            self.log_message.emit(f"Serial read error: {e}")

    def switch_channel(self):
        """Alternate reading between two chips."""
        if self.current_time - self.last_switch_time < self.switch_speed:
            return
        self.last_switch_time = self.current_time
        # Get previous op_mode settings
        c1_mode = self.c1_settings.op_mode
        c2_mode = self.c2_settings.op_mode

        # Exchange new op_mode settings
        self.write_serial(self.c1_settings.update("op_mode", 0))
        self.write_serial(self.c2_settings.update("op_mode", 0))  # Turn off before switching
        time.sleep(0.01)
        self.write_serial(self.c1_settings.update("op_mode", c2_mode))
        self.write_serial(self.c2_settings.update("op_mode", c1_mode))

        # Emit UI update signal
        self.settings_changed.emit()
    
    def simulate_data(self):
        """Generate simulated ADC data at the configured frequency."""
        t = self.current_time
        v1 = 512 + 400 * math.sin(2 * math.pi * 1.0 * t) + 20 * math.sin(t * 10)
        v2 = 512 + 150 * math.cos(2 * math.pi * 0.5 * t) + 10 * math.cos(t * 7)
        self.buffer.append([v1, v2], self.current_time)
        self.active_buffer.append([True, True], self.current_time)

    def refresh_port(self):
        port_list = []
        ports = serial.tools.list_ports.comports()
        for port in ports:
            port_list.append(port.device)
        # ensure simulation option exists
        port_list.append("SIMULATE")
        return port_list
    
    def disconnect(self):
        """Stop the worker thread gracefully."""
        if self.running:
            self.running = False
            self.simulate = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except Exception:
                    self.log_message.emit("Port already disconnected.")
                    pass
            self.log_message.emit("Stopped.")

    def connect(self, serial_port):
        if serial_port == "SIMULATE":
            self.serial_port = None
            self.running = True
            self.simulate = True
            self.log_message.emit("Simulation started.")
        else:
            # Try to connect to serial port
            try:
                self.serial_port = serial.Serial(serial_port, baudrate=115200, timeout=1)
                self.running = True
                self.simulate = False
                self.log_message.emit(f"Connected to {serial_port}.")
                time.sleep(0.01)
                self.write_serial(self.c1_settings.update_all())
                self.write_serial(self.c2_settings.update_all())
            except Exception as e:
                self.serial_port = None
                self.log_message.emit(f"Error connecting: {e}")
    
    def is_running(self):
        return self.running

    def clear(self):
        self.buffer.clear()
        self.active_buffer.clear()

    def get(self, convert=False, active=False):
        timestamp, value = self.buffer.get()
        if len(timestamp) == 0:
            return [], [], []
        v1 = value[:, 0]
        v2 = value[:, 1]

        if convert:
            z1 = LMP91000_VREF * self.c1_settings.z
            z2 = LMP91000_VREF * self.c2_settings.z
            v1 = (v1 * ADC_VSTEP - z1) / self.c1_settings.gain * 1e3  # uA
            v2 = (v2 * ADC_VSTEP - z2) / self.c2_settings.gain * 1e3  # uA

        if active:
            _, active_value = self.active_buffer.get()
            c1 = active_value[:, 0]
            c2 = active_value[:, 1]
            v1[~c1] = numpy.nan
            v2[~c2] = numpy.nan

        return timestamp, v1, v2
    
    def get_axis_limit(self, convert=False):
        if convert:
            z1 = LMP91000_VREF * self.c1_settings.z
            z2 = LMP91000_VREF * self.c2_settings.z
            ymin1 = (-ADC_VREF * 0.05 - z1) / self.c1_settings.gain * 1e3  # uA
            ymax1 = (ADC_VREF * 1.05 - z1) / self.c1_settings.gain * 1e3  # uA
            ymin2 = (-ADC_VREF * 0.05 - z2) / self.c2_settings.gain * 1e3  # uA
            ymax2 = (ADC_VREF * 1.05 - z2) / self.c2_settings.gain * 1e3  # uA
        else:
            ymin1 = ADC_YMIN
            ymax1 = ADC_YMAX
            ymin2 = ADC_YMIN
            ymax2 = ADC_YMAX
        xmin = self.current_time - self.buffer_length
        xmax = self.current_time
        return (xmin, xmax), (ymin1, ymax1), (ymin2, ymax2)
    
    def set_switch_settings(self, enabled, speed=1):
        self.switch = enabled
        self.switch_speed = speed

class LMP91000UI(QWidget):
    # Signals: emit data and status updates safely to UI
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LMP91000 UI")
        self.resize(800, 1000)
        
        # --- Buffer and state (UI-side) ---
        self.adc_sampling_freq = ADC_SAMPLING_FREQ # Hz
        self.dt = 1.0 / self.adc_sampling_freq
        self.buffer_length = BUFFER_LENGTH # s

        # --- Controls ---
        self.combine = False
        self.convert = False
        self.switch = False
        self.active = False
        
        # --- Serial worker ---
        self.worker = LMP91000SerialWorker(self.adc_sampling_freq, self.buffer_length)
        
        self.worker.settings_changed.connect(self.on_worker_settings_changed)
        self.worker.log_message.connect(self.on_worker_log_message)
       
        self.worker.start()

        # --- Layouts ---
        layout = QVBoxLayout()
        self.setLayout(layout)

        # --- Serial Controls ---
        serial_layout = QHBoxLayout()
        layout.addLayout(serial_layout)

        serial_port_group = QGroupBox("Serial Port")
        serial_port_layout = QVBoxLayout()
        serial_port_group.setLayout(serial_port_layout)

        serial_layout.addWidget(serial_port_group)
        self.port_combo = QComboBox()
        serial_port_layout.addWidget(self.port_combo)
        # always allow a simulation port for testing

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.on_refresh_clicked)
        serial_port_layout.addWidget(self.refresh_button)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect_toggled)
        serial_port_layout.addWidget(self.connect_button)
        
        # --- Log Output ---
        serial_log_group = QGroupBox("Log Output")
        serial_layout.addWidget(serial_log_group)
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        serial_log_group.setLayout(QVBoxLayout())
        serial_log_group.layout().addWidget(self.log_output)

        # --- Register Settings for Chip 1 & 2 ---
        @dataclass
        class LMP91000SettingsUI:
            # UI elements
            chip: int = 0
            group: QGroupBox = None
            tia_gain_combo: QComboBox = None
            rload_combo: QComboBox = None
            ref_source_combo: QComboBox = None
            int_z_combo: QComboBox = None
            bias_sign_combo: QComboBox = None
            bias_combo: QComboBox = None
            fet_short_combo: QComboBox = None
            op_mode_combo: QComboBox = None

        chip_setting_layout = QHBoxLayout()
        layout.addLayout(chip_setting_layout)

        self.c1_settings_ui = LMP91000SettingsUI(chip=1, group=QGroupBox("Chip 1"))
        self.c2_settings_ui = LMP91000SettingsUI(chip=2, group=QGroupBox("Chip 2"))
        
        for settings_ui, settings in [
            (self.c1_settings_ui, self.worker.c1_settings),
            (self.c2_settings_ui, self.worker.c2_settings)]:
            chip_setting_layout.addWidget(settings_ui.group)
            settings_layout = QHBoxLayout()
            settings_ui.group.setLayout(settings_layout)

            TIACN_group = QGroupBox("TIA")
            TIACN_layout = QGridLayout()
            TIACN_group.setLayout(TIACN_layout)
            settings_layout.addWidget(TIACN_group)

            settings_ui.tia_gain_combo = QComboBox()
            TIACN_layout.addWidget(QLabel("TIA gain:"), 0, 0)
            TIACN_layout.addWidget(settings_ui.tia_gain_combo, 1, 0)
            settings_ui.tia_gain_combo.addItems(settings.get_all("tia_gain"))
            settings_ui.tia_gain_combo.setCurrentIndex(settings.tia_gain)
            settings_ui.tia_gain_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "tia_gain"))

            settings_ui.rload_combo = QComboBox()
            TIACN_layout.addWidget(QLabel("R_load:"), 2, 0)
            TIACN_layout.addWidget(settings_ui.rload_combo, 3, 0)
            settings_ui.rload_combo.addItems(settings.get_all("rload"))
            settings_ui.rload_combo.setCurrentIndex(settings.rload)
            settings_ui.rload_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "rload"))

            REFCN_group = QGroupBox("Reference")
            REFCN_layout = QGridLayout()
            REFCN_group.setLayout(REFCN_layout)
            settings_layout.addWidget(REFCN_group)

            settings_ui.ref_source_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Reference:"), 0, 0)
            REFCN_layout.addWidget(settings_ui.ref_source_combo, 0, 1)
            settings_ui.ref_source_combo.addItems(settings.get_all("ref_source"))
            settings_ui.ref_source_combo.setCurrentIndex(settings.ref_source)
            settings_ui.ref_source_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "ref_source"))

            settings_ui.int_z_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Internal zero:"), 1, 0)
            REFCN_layout.addWidget(settings_ui.int_z_combo, 1, 1)
            settings_ui.int_z_combo.addItems(settings.get_all("int_z"))
            settings_ui.int_z_combo.setCurrentIndex(settings.int_z)
            settings_ui.int_z_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "int_z"))

            settings_ui.bias_sign_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Bias polarity:"), 2, 0)
            REFCN_layout.addWidget(settings_ui.bias_sign_combo, 2, 1)
            settings_ui.bias_sign_combo.addItems(settings.get_all("bias_sign"))
            settings_ui.bias_sign_combo.setCurrentIndex(settings.bias_sign)
            settings_ui.bias_sign_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "bias_sign"))

            settings_ui.bias_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Bias level:"), 3, 0)
            REFCN_layout.addWidget(settings_ui.bias_combo, 3, 1)
            settings_ui.bias_combo.addItems(settings.get_all("bias"))
            settings_ui.bias_combo.setCurrentIndex(settings.bias)
            settings_ui.bias_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "bias"))

            MODECN_group = QGroupBox("Mode")
            MODECN_layout = QGridLayout()
            MODECN_group.setLayout(MODECN_layout)
            settings_layout.addWidget(MODECN_group)

            settings_ui.fet_short_combo = QComboBox()
            MODECN_layout.addWidget(QLabel("FET short:"), 0, 0)
            MODECN_layout.addWidget(settings_ui.fet_short_combo, 1, 0)
            settings_ui.fet_short_combo.addItems(settings.get_all("fet_short"))
            settings_ui.fet_short_combo.setCurrentIndex(settings.fet_short)
            settings_ui.fet_short_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "fet_short"))

            settings_ui.op_mode_combo = QComboBox()
            MODECN_layout.addWidget(QLabel("Operating mode:"), 2, 0)
            MODECN_layout.addWidget(settings_ui.op_mode_combo, 3, 0)
            settings_ui.op_mode_combo.addItems(settings.get_all("op_mode"))
            settings_ui.op_mode_combo.setCurrentIndex(settings.op_mode)
            settings_ui.op_mode_combo.currentIndexChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings_ui.chip, "op_mode"))

        # --- Auto mode control ---
        switch_group = QGroupBox("Auto Switching")
        switch_layout = QVBoxLayout()
        switch_layout.setAlignment(Qt.AlignTop)
        switch_group.setLayout(switch_layout)
        self.c2_settings_ui.group.layout().addWidget(switch_group) # Add two chip-2 panel

        switch_layout.addWidget(QLabel("Speed (s):"))
        self.switch_speed_combo = QComboBox()
        self.switch_speed_combo.addItems(["0.5", "1", "5", "10", "30", "60", "120", "300"])
        self.switch_speed_combo.setCurrentText("1")
        switch_layout.addWidget(self.switch_speed_combo)
        self.switch_speed_combo.currentIndexChanged.connect(self.on_switch_speed_changed)

        self.switch_enable_button = QPushButton("Enable")
        switch_layout.addWidget(self.switch_enable_button)
        self.switch_enable_button.clicked.connect(self.on_switch_enable_toggled)
        
        # --- Plot ---
        readout_group = QGroupBox("ADC Readout")
        layout.addWidget(readout_group)
        readout_layout = QVBoxLayout()
        readout_group.setLayout(readout_layout)

        readout_button_layout = QHBoxLayout()
        readout_layout.addLayout(readout_button_layout)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.on_clear_clicked)
        readout_button_layout.addWidget(self.clear_button)

        self.save_button = QPushButton("Save Data")
        self.save_button.clicked.connect(self.on_save_clicked)
        readout_button_layout.addWidget(self.save_button)

        self.save_image_button = QPushButton("Save Image")
        self.save_image_button.clicked.connect(self.on_save_image_clicked)
        readout_button_layout.addWidget(self.save_image_button)

        self.combine_button = QPushButton("Combine")
        self.combine_button.clicked.connect(self.on_combine_toggled)
        readout_button_layout.addWidget(self.combine_button)

        self.active_button = QPushButton("Show Active")
        self.active_button.clicked.connect(self.on_active_toggled)
        readout_button_layout.addWidget(self.active_button)

        self.auto_button = QPushButton("Auto Scale")
        self.auto_button.clicked.connect(self.on_auto_clicked)
        readout_button_layout.addWidget(self.auto_button)

        self.convert_button = QPushButton("Convert Current")
        self.convert_button.clicked.connect(self.on_convert_toggled)
        readout_button_layout.addWidget(self.convert_button)

        # --- Plotting Area with PyQtGraph (two PlotItems via GraphicsLayoutWidget) ---
        self.plot_widget = pg.GraphicsLayoutWidget()
        readout_layout.addWidget(self.plot_widget)
        self.plot_widget.setBackground("w")

        # Top plot (channel 1 or combined)
        self.plot_item1 = self.plot_widget.addPlot(row=0, col=0)
        self.plot_item1.setLabel("left", "Voltage (V)")
        self.plot_item1.setLabel("bottom", "Time (s)")
        self.plot_item1.showGrid(x=True, y=True)
        self.plot_item1.addLegend()

        # Bottom plot (channel 2)
        self.plot_item2 = self.plot_widget.addPlot(row=1, col=0)
        self.plot_item2.setLabel("left", "Voltage (V)")
        self.plot_item2.setLabel("bottom", "Time (s)")
        self.plot_item2.showGrid(x=True, y=True)

        # Disable all mouse interactions on both view boxes (pan/zoom/menus)
        for pi in (self.plot_item1, self.plot_item2):
            vb = pi.getViewBox()
            #vb.setMouseEnabled(x=False, y=True)
            vb.setMenuEnabled(False)

        # plot curves: separate curve per plot item
        self.plot_adc1 = self.plot_item1.plot([], [], pen=pg.mkPen("b", width=2), name="Chip 1")
        self.plot_adc2 = self.plot_item1.plot([], [], pen=pg.mkPen("r", width=2), name="Chip 2")
        self.plot2_adc2 = self.plot_item2.plot([], [], pen=pg.mkPen("r", width=2), name="Chip 2")
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        # aim to refresh roughly at sampling period
        self.timer.start(int(1000 / REFRESH_RATE))
        
        self.auto_button.click()
        self.refresh_button.click()

    def on_refresh_clicked(self):
        port_list = self.worker.refresh_port()
        self.port_combo.clear()
        self.port_combo.addItems(port_list)
        self.log_output.append("Port list refreshed.")

    def on_connect_toggled(self):
        """Toggle connection: connect to selected port or disconnect if already connected."""
        if self.worker.is_running():
            # currently connected/running -> disconnect
            self.worker.disconnect()
            self.connect_button.setText("Connect")
        else:
            # attempt connect using selected port (allow empty selection)
            port = self.port_combo.currentText() if self.port_combo.count() > 0 else ""
            if not port:
                self.log_output.append("No serial port selected.")
                return
            self.worker.connect(port)
            self.connect_button.setText("Disconnect")

    def on_worker_settings_changed(self):
        self.worker.blockSignals(True)  # Prevent signal loop
        for settings_ui, settings in [
            (self.c1_settings_ui, self.worker.c1_settings),
            (self.c2_settings_ui, self.worker.c2_settings)]:
            settings_ui.tia_gain_combo.setCurrentIndex(settings.tia_gain)
            settings_ui.rload_combo.setCurrentIndex(settings.rload)
            settings_ui.ref_source_combo.setCurrentIndex(settings.ref_source)
            settings_ui.int_z_combo.setCurrentIndex(settings.int_z)
            settings_ui.bias_sign_combo.setCurrentIndex(settings.bias_sign)
            settings_ui.bias_combo.setCurrentIndex(settings.bias)
            settings_ui.fet_short_combo.setCurrentIndex(settings.fet_short)
            settings_ui.op_mode_combo.setCurrentIndex(settings.op_mode)
        self.worker.blockSignals(False)

    def on_worker_log_message(self, msg):
        """Slot: receive log message from worker and display in UI."""
        self.log_output.append("<Serial> " + msg)

    def on_switch_speed_changed(self):
        switch_speed = float(self.switch_speed_combo.currentText())
        self.worker.set_switch_settings(self.switch, switch_speed)
        if self.switch:
            self.log_output.append(f"Changed auto switching speed to {switch_speed} s")

    def on_switch_enable_toggled(self):
        if self.worker.is_running():
            self.switch = not self.switch
        else:
            self.switch = False
            return
        
        if self.switch:
            switch_speed = float(self.switch_speed_combo.currentText())
            self.worker.set_switch_settings(True, switch_speed)
            self.switch_enable_button.setText("Disable")
            for settings in [self.c1_settings_ui, self.c2_settings_ui]:
                settings.op_mode_combo.setEnabled(False)
            self.log_output.append(f"Enabled auto switching every {switch_speed} s.")
        else:
            self.switch_enable_button.setText("Enable")
            self.worker.set_switch_settings(False)
            for settings in [self.c1_settings_ui, self.c2_settings_ui]:
                settings.op_mode_combo.setEnabled(True)
            self.log_output.append("Disabled auto switching.")

    def on_clear_clicked(self):
        self.worker.clear()
        self.log_output.append("Buffer cleared.")

    def on_save_clicked(self):
        t, v1, v2 = self.worker.get(self.convert, self.active) # Save all data
        fname, _ = QFileDialog.getSaveFileName(self, "Save Data", "adc_data.csv", "CSV Files (*.csv)")
        if not fname:
            return
        with open(fname, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["t", "adc1", "adc2"])
            for i in range(len(t)):
                writer.writerow([t[i], v1[i], v2[i]])
        self.log_output.append(f"Data saved to {fname}")

    def on_save_image_clicked(self):
        pix = self.plot_widget.grab()
        fname, _ = QFileDialog.getSaveFileName(self, "Save Image", "adc_plot.png", "PNG Files (*.png)")
        if not fname:
            return
        pix.save(fname)
        self.log_output.append(f"Image saved to {fname}")

    def on_combine_toggled(self):
        self.combine = not self.combine
        if self.combine:
            self.combine_button.setText("Split")
            # show both channels in the top plot and hide the bottom plot
            try:
                self.plot_item2.hide()
            except Exception:
                pass
            # keep top plot showing both curves
            self.plot_adc2.show()
        else:
            self.combine_button.setText("Combine")
            try:
                self.plot_item2.show()
            except Exception:
                pass
            # clear the extra top trace for the separate mode (we"ll plot channel2 in bottom)
            self.plot_adc2.setData([])

    def on_active_toggled(self):
        self.active = not self.active

        if self.active:
            self.active_button.setText("Show All")
        else:
            self.active_button.setText("Show Active")

    def on_auto_clicked(self):
        _, ylim1, ylim2 = self.worker.get_axis_limit(self.convert)
        if self.combine:
            self.plot_item1.setYRange(min(ylim1[0], ylim2[0]), max(ylim1[1], ylim2[1]), padding=0)
        else:
            self.plot_item1.setYRange(ylim1[0], ylim1[1], padding=0)
            self.plot_item2.setYRange(ylim2[0], ylim2[1], padding=0)
        self.plot_item1.enableAutoRange(axis="x")
        self.plot_item2.enableAutoRange(axis="x")

    def on_convert_toggled(self):
        self.convert = not self.convert
        if self.convert:
            self.convert_button.setText("Show Actual")
            self.plot_item1.setLabel("left", "Current (uA)")
            self.plot_item2.setLabel("left", "Current (uA)")
        else:
            self.convert_button.setText("Show Current")
            self.plot_item1.setLabel("left", "ADC Value")
            self.plot_item2.setLabel("left", "ADC Value")
        
        self.auto_button.click()

    def update_plot(self):
        timestamp, v1, v2 = self.worker.get(self.convert, self.active)
        
        if self.combine:
            self.plot_adc1.setData(timestamp, v1)
            self.plot_adc2.setData(timestamp, v2)
        else:
            self.plot_adc1.setData(timestamp, v1)
            self.plot2_adc2.setData(timestamp, v2)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = LMP91000UI()
    gui.show()
    sys.exit(app.exec_())
