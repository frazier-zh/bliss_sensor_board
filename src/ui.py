"""
    Project: BLISS
    Application: BLISS Sensor Board User Interface
    File: src/ui.py
    Description: Main UI and logic.
    Author: Fang Zihang (Dr.)
    Email: fang.zh@nus.edu.sg
    Affiliation: National University of Singapore
"""
# --- Python packages ---
import sys
import serial
import serial.tools.list_ports
import re
import math
import time
import csv
from functools import partial

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QComboBox, QTextEdit, QGridLayout,
    QGroupBox, QFileDialog
)
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QPointF
import pyqtgraph as pg

# --- CONSTANTS ---
# * DO NOT CHANGE *
ADC_SAMPLE_RATE = 20
N_CHANNELS = 4

# * OK TO CHANGE *
DEFAULT_SIZE = 300  # 5 mins
MEMORY_SIZE = 360000
REFRESH_RATE = 30

PLOT_MAX_POINTS = 6000
MASTER_MAX_POINTS = 1024
MASTER_MIN_SIZE = 10  # 10 seconds
MASTER_MAX_SIZE = 1800 # 30 mins
MASTER_REFRESH_RATE = 1

#COLORS = ["r", "b", "g", "m"]
COLORS = ["y", "c", "r", "g"]
#BACKGROUD_COLOR = "w"
BACKGROUD_COLOR = "k"
#SECONDARY_COLOR = "k"
SECONDARY_COLOR = "w"
LINEWIDTH = 2
PLOT_AXIS_STYLE = {
    "top": {
        "style": { "showValues": False,},
    },
    "bottom": {
        "style": { "showValues": False,},
    },
    "left": {
        "width": 20,
        "style": { "showValues": False,},
    },
    "right": {
        "width": 40,
        "style": { "showValues": True,},
    },
}
MASTER_PLOT_HEIGHT = 60
MASTER_AXIS_STYLE = {
    "top": {
        "height": 10,
        "style": { "showValues": False,},
    },
    "bottom": {
        "height": 10,
        "style": { "showValues": False,},
    },
    "left": {
        "width": 150,
        "style": { "showValues": False,},
    },
    "right": {
        "width": 150,
        "style": { "showValues": False,},
    },
}

def combine_images_vertical(pixmaps):
    total_height = sum(p.height() for p in pixmaps)
    max_width = max(p.width() for p in pixmaps)

    result = QPixmap(max_width, total_height)
    result.fill(pg.mkColor(BACKGROUD_COLOR))
    painter = QPainter(result)
    y_offset = 0
    for p in pixmaps:
        painter.drawPixmap(0, y_offset, p)
        y_offset += p.height()
    painter.end()
    return result

# --- Local python packages ---
from lib import storage
from plot import Plot, MasterPlot
from device import LMP91000, PHSensor

# --- Serial worker ---
class SerialWorker(QThread):
    """
    Worker thread for serial I/O and simulation. Runs independently and emits signals
    to safely update the UI without blocking the main thread.
    """
    # Outbound signals
    log_message = pyqtSignal(str)           # Write to UI from serial
    # Inbound signals
    write_serial = pyqtSignal(str)          # Write to serial
    clear_buffer = pyqtSignal()

    def __init__(self, n_channels, channel_formulas):
        super().__init__()
        self.n_channels = n_channels
        self.channel_formulas = channel_formulas

        self.ADC_SAMPLE_RATE = ADC_SAMPLE_RATE
        self.adc_dt = 1.0 / self.ADC_SAMPLE_RATE
        self.adc_dt_half = self.adc_dt / 2

        self.serial_read_pattern = r"ADC" + r" (\d+)" * self.n_channels

        self.memory_size = MEMORY_SIZE
        self.memory = storage(n_channels=self.n_channels, size=self.memory_size)
        self.get = self.memory.get
        self.at = self.memory.at

        self.start_time = 0
        self.current_time = 0
        self.simulate = False
        self.serial_port = None
        self.running = False

        self.write_serial.connect(self.on_write_serial)
        self.clear_buffer.connect(self.clear)
        
    def run(self):
        """Main thread loop: read serial or simulate data at sampling frequency."""
        self.start_time = time.time()
        while True:
            self.current_time = time.time() - self.start_time
            if not self.running:
                self.memory.append(0, self.current_time)
                time.sleep(0.01)
                continue
            if self.simulate:
                self.simulate_data()
                time.sleep(self.adc_dt)
            else:
                self.read_serial()
                time.sleep(self.adc_dt_half)

    def on_write_serial(self, data: str):
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
            
            match = re.search(self.serial_read_pattern, line)
            if match:
                v = [self.channel_formulas[i](int(match.group(i+1))) for i in range(self.n_channels)]
                self.memory.append(v, self.current_time)
            else:
                # non-ADC lines go to log
                self.log_message.emit(line)
        except Exception as e:
            self.log_message.emit(f"Serial read error: {e}")
    
    def simulate_data(self):
        """Generate simulated ADC data at the configured frequency."""
        t = self.current_time
        raw_data = [
            500 + 400 * math.sin(2 * math.pi * 1.0 * t) + 20 * math.sin(t * 10),
            500 + 150 * math.cos(2 * math.pi * 0.5 * t) + 10 * math.cos(t * 7),
            500 + 250 * math.copysign(1, math.cos(2 * math.pi * 0.2 * t)),
            500 + 200 * math.copysign(1, math.cos(2 * math.pi * 0.2 * (t + 2))),
        ]
        v = [self.channel_formulas[i](raw_data[i]) for i in range(self.n_channels)]
        self.memory.append(v, self.current_time)

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
        if not self.running:
            return
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
            return
        # Try to connect to serial port
        try:
            self.serial_port = serial.Serial(serial_port, baudrate=115200, timeout=1)
            self.running = True
            self.simulate = False
            self.log_message.emit(f"Connected to {serial_port}.")
            time.sleep(0.01)
        except Exception as e:
            self.serial_port = None
            self.log_message.emit(f"Error connecting: {e}")
    
    def is_running(self):
        return self.running

    def clear(self):
        self.memory.clear()
        self.start_time = time.time()
        self.current_time = 0

class SettingGroup(QGroupBox):
    # Helper class for auto-generating Qgroup with QComboBox
    # For sensor configuration
    write_serial = pyqtSignal(str)
    auto_yrange = pyqtSignal()

    def __init__(self, name, setting):
        super().__init__(name)
        layout = QGridLayout()
        self.setLayout(layout)
        self.setting = setting

        if not hasattr(setting, "active_keys"):
            return
        
        def func(key, idx):
            self.write_serial.emit(self.setting.update(key, idx))
            self.auto_yrange.emit()

        row = 0
        for key in self.setting.active_keys:
            label_str = getattr(self.setting, key + "_label")
            layout.addWidget(QLabel(label_str + ":"), row, 0)
            combo = QComboBox()
            layout.addWidget(combo, row, 1)
            
            combo.addItems(getattr(self.setting, key + "_text"))
            combo.setCurrentIndex(getattr(self.setting, key))
            combo.currentIndexChanged.connect(partial(func, key))
            row = row + 1

class BLISSUI(QWidget):
    # Signals: emit data and status updates safely to UI
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLISS Sensor Interface")
        self.resize(1200, 800)
        
        # --- Settings ---
        self.n_channels = N_CHANNELS
        self.settings = [LMP91000(id=i+1) for i in range(3)] + [PHSensor(id=4)]
        
        # --- Serial worker ---
        self.worker = SerialWorker(self.n_channels, [settings.formula for settings in self.settings])
        self.worker.log_message.connect(self.on_worker_log_message)
        self.worker.start()

        # --- Layouts ---
        layout = QGridLayout()
        self.setLayout(layout)
        layout.setVerticalSpacing(0)
        
        plot_layout = QVBoxLayout()
        plot_layout.setSpacing(0)
        layout.addLayout(plot_layout, 0, 0)
        
        serial_port_group = QGroupBox("Serial Port")
        layout.addWidget(serial_port_group, 0, 1,)

        self.log_output = QTextEdit()
        layout.addWidget(self.log_output, self.n_channels+1, 0, 1, 2)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 0)

        # --- Master plot ---
        self.live = True
        self.plot_update = True
        self.live_size = DEFAULT_SIZE
        self.live_offset = 0
        self.master_plot = MasterPlot(
            n_channels=self.n_channels,
            min_size=MASTER_MIN_SIZE,
            max_size=MASTER_MAX_SIZE,
            colors=COLORS,
            color2=SECONDARY_COLOR,
        )
        self.master_plot.setBackground(BACKGROUD_COLOR)
        self.master_plot.setFixedHeight(MASTER_PLOT_HEIGHT)
        self.master_plot.set_axis_style(MASTER_AXIS_STYLE)
        self.master_max_points = MASTER_MAX_POINTS
        self.master_plot.region_changed.connect(self.on_region_changed)

        # --- Plot item ---
        self.plots = [Plot(color=COLORS[i], linewidth=LINEWIDTH, color2=SECONDARY_COLOR) for i in range(self.n_channels)]
        self.plot_max_points = PLOT_MAX_POINTS
        self.mouse_pos = QPointF(0, 0)
        self.mouse_update = True
        for i in range(self.n_channels):
            layout.addWidget(self.plots[i], i+1, 0)
            self.plots[i].setBackground(BACKGROUD_COLOR)
            self.plots[i].set_axis_style(PLOT_AXIS_STYLE)
            self.plots[i].showGrid(x=True, y=True, alpha=0.2)
            self.plots[i].scene().sigMouseMoved.connect(self.on_plot_mouse_moved)
        self.plots[-1].getAxis("bottom").setStyle(showValues=True)
        self.plots[-1].setLabel("bottom", "Time (s)")
        
        # --- Serial controls ---
        serial_port_layout = QVBoxLayout()
        serial_port_group.setLayout(serial_port_layout)

        self.port_combo = QComboBox()
        serial_port_layout.addWidget(self.port_combo)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.on_refresh_clicked)
        serial_port_layout.addWidget(self.refresh_button)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.on_connect_toggled)
        serial_port_layout.addWidget(self.connect_button)
        
        # --- Log output ---
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumHeight(180)
        
        # --- Plot control layout ---
        plot_control = QWidget()
        plot_control_layout = QHBoxLayout()
        plot_control.setLayout(plot_control_layout)
        plot_layout.addWidget(plot_control)
        plot_layout.addWidget(self.master_plot)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.on_clear_clicked)
        plot_control_layout.addWidget(self.clear_button)

        self.save_button = QPushButton("Save Data")
        self.save_button.clicked.connect(self.on_save_clicked)
        plot_control_layout.addWidget(self.save_button)

        self.save_image_button = QPushButton("Save Image")
        self.save_image_button.clicked.connect(self.on_save_image_clicked)
        plot_control_layout.addWidget(self.save_image_button)

        self.auto_button = QPushButton("Auto")
        self.auto_button.clicked.connect(self.on_auto_clicked)
        plot_control_layout.addWidget(self.auto_button)

        self.live_button = QPushButton("Show Live")
        # live option is globally synced by live_button and master_plot
        self.live_button.clicked.connect(self.on_live_clicked)
        plot_control_layout.addWidget(self.live_button)

        # --- Register settings ---
        self.setting_groups = [SettingGroup(f"Channel {i+1}", self.settings[i]) for i in range(self.n_channels)]
        for i in range(self.n_channels):
            layout.addWidget(self.setting_groups[i], i+1, 1)
            self.setting_groups[i].write_serial.connect(self.worker.write_serial)
            self.setting_groups[i].auto_yrange.connect(partial(self.on_auto_yaxis, i))

        # --- Start main logic ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / REFRESH_RATE))

        self.master_timer = QTimer(self)
        self.master_timer.timeout.connect(self.update_master)
        self.master_timer.start(int(1000 / MASTER_REFRESH_RATE))

        self.refresh_button.click()
        self.auto_button.click()

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
            self.connect_button.setText("Connect")
            # attempt connect using selected port (allow empty selection)
            port = self.port_combo.currentText() if self.port_combo.count() > 0 else ""
            if not port:
                self.log_output.append("No serial port selected.")
                return
            self.worker.connect(port)
            self.connect_button.setText("Disconnect")
            # write all register settings
            for setting in self.settings:
                self.worker.write_serial.emit(setting.update_all())
            self.auto_button.click()
            self.live_button.click()

    def on_worker_log_message(self, msg):
        """Slot: receive log message from worker and display in UI."""
        self.log_output.append("<Serial> " + msg)

    def on_clear_clicked(self):
        self.worker.clear_buffer.emit()
        self.log_output.append("Buffer cleared.")

    def on_save_clicked(self):
        timestamp, value = self.worker.get() # Save all data
        fname, _ = QFileDialog.getSaveFileName(self, "Save Data", "adc_data.csv", "CSV Files (*.csv)")
        if not fname:
            return
        with open(fname, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["t", "adc1", "adc2"])
            for i in range(len(timestamp)):
                writer.writerow([timestamp[i], value[i, 0], value[i, 1]])
        self.log_output.append(f"Data saved to {fname}")

    def on_save_image_clicked(self):
        pix = combine_images_vertical([self.master_plot.grab()] + [plot.grab() for plot in self.plots])
        fname, _ = QFileDialog.getSaveFileName(self, "Save Image", "adc_plot.png", "PNG Files (*.png)")
        if not fname:
            return
        pix.save(fname)
        self.log_output.append(f"Image saved to {fname}")

    def on_auto_yaxis(self, idx):
        self.plots[idx].setLabel("left", self.settings[idx].get_ylabel())
        self.plots[idx].setYRange(*self.settings[idx].get_yrange())

    def on_auto_clicked(self):
        for i in range(self.n_channels):
            self.on_auto_yaxis(i)

    def on_live_clicked(self):
        self.live_size = self.master_plot.region_size
        self.live_offset = 0
        self.live = True

    def on_region_changed(self, xmin, xmax):
        # Activates when user manually change the region
        # Live logic:
        # - When graph is live
        # -- The new region contains the last point, the graph is live
        # -- The graph is not live if otherwise
        # - When graph is not live, it stays to be not live
        self.live_size = xmax - xmin
        if self.live:
            if self.master_plot.xmax < xmax and self.master_plot.xmax > xmin:
                self.live_offset = xmax - self.master_plot.xmax
                self.live = True
            else:
                self.live = False
        else:
            self.live = False
        self.plot_update = True

    def on_plot_mouse_moved(self, pos):
        self.mouse_pos = pos
        self.mouse_update = True

    def update(self):
        # update plot value
        current = self.worker.current_time
        if self.live:
            # when view is live
            end = current + self.live_offset
            start = end - self.live_size
            self.master_plot.try_set_region(start, end)
            timestamp, value = self.worker.get(start, end, max_points=self.plot_max_points)
            if not len(timestamp) == 0:
                for i in range(self.n_channels):
                    self.plots[i].set_region(start, end)
                    self.plots[i].set_data(timestamp, value[:, i])
        else:
            # when view region has changed
            start, end = self.master_plot.get_region()
            if self.plot_update or start < current < end:
                timestamp, value = self.worker.get(start, end, max_points=self.plot_max_points)
                if not len(timestamp) == 0:
                    for i in range(self.n_channels):
                        self.plots[i].set_region(start, end)
                        self.plots[i].set_data(timestamp, value[:, i])
        
        # update last value
        last = self.worker.at()
        for i in range(self.n_channels):
            self.plots[i].set_last(last[i], f"X: {current:.1f}\nY: {last[i]:.2f}")

        # update mouse line
        mouse_pos = self.plots[0].vb.mapSceneToView(self.mouse_pos)
        x = mouse_pos.x()
        mouse_data = self.worker.at(x)
        for i in range(self.n_channels):
            self.plots[i].set_line(x, f"X  : {x:.1f}\nΔX: {current-x:.1f}\nY  : {mouse_data[i]:.2f}")
        
        # reset update indicator
        self.mouse_update = False
        self.plot_update = False

    def update_master(self):
        timestamp, value = self.worker.get(max_points=self.master_max_points)
        if not len(timestamp) == 0:
            self.master_plot.set_data(timestamp, value)
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = BLISSUI()
    gui.show()
    sys.exit(app.exec_())
