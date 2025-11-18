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

ADC_SAMPLING_FREQ = 20
BUFFER_LENGTH = 12 * 60 * 60 # (s)
ADC_RESOLUTION = 10
ADC_N = (1 << ADC_RESOLUTION) - 1

LMP91000_VREF = 3.0
ADC_VREF = 3.3
ADC_VSTEP = ADC_VREF / ADC_N

ADC_YMIN = -100
ADC_YMAX = ADC_N + 100
REFRESH_RATE = 60

@dataclass
class LMP91000REGS:
    ## TIACN -- TIA control register (address 0x10)
    TIACN_REG = 0x10
    # Transimpedance amplifier gain settings 
    TIA_GAIN_REG = TIACN_REG
    TIA_GAIN_MASK = 0b00011100
    TIA_GAIN_SHIFT = 2
    TIA_GAIN_BIN = {
        "External": 0b000,
        "2.75 kΩ": 0b001,
        "3.5 kΩ": 0b010,
        "7 kΩ": 0b011,
        "14 kΩ": 0b100,
        "35 kΩ": 0b101,
        "120 kΩ": 0b110,
        "350 kΩ": 0b111,
    }
    TIA_GAIN_DEFAULT = "120 kΩ"

    # Load resistance settings
    RLOAD_ADDE = TIACN_REG
    RLOAD_MASK = 0b00000011
    RLOAD_SHIFT = 0
    RLOAD_BIN = {
        "10 Ω": 0b00,
        "33 Ω": 0b01,
        "50 Ω": 0b10,
        "100 Ω": 0b11,
    }
    RLOAD_DEFAULT = "100 Ω"

    ## REFCN -- Reference control register (address 0x11)
    REFCN_REG = 0x11
    # Reference voltage source selection
    REF_SOURCE_REG = REFCN_REG
    REF_SOURCE_MASK = 0b10000000
    REF_SOURCE_SHIFT = 7
    REF_SOURCE_BIN = {
        "Internal": 0b0,
        "External": 0b1,
    }
    REF_SOURCE_DEFAULT = "External"

    # Internal zero selection, percentage of the source reference
    INT_Z_REG = REFCN_REG
    INT_Z_MASK = 0b01100000
    INT_Z_SHIFT = 5
    INT_Z_BIN = {
        "0.6 V": 0b00, # 20%
        "1.5 V": 0b01, # 50%
        "2.0 V": 0b10, # 67%
        "Bypass": 0b11,  # Internal zero circuitry bypassed, only in O2 ground referred measurements
    }
    INT_Z_DEFAULT = "1.5 V"

    # Bias polarity selection, (V_WE - V_RE) sign
    BIAS_SIGN_REG = REFCN_REG
    BIAS_SIGN_MASK = 0b00010000
    BIAS_SIGN_SHIFT = 4
    BIAS_SIGN_BIN = {
        "Negative WE<RE": 0b0,
        "Positive WE>RE": 0b1,
    }
    BIAS_SIGN_DEFAULT = "Positive WE>RE"

    # Bias voltage level, percentage of the source reference
    BIAS_REG = REFCN_REG
    BIAS_MASK = 0b00001111
    BIAS_SHIFT = 0
    BIAS_BIN = {
        "0.00 V": 0b0000, # 0%
        "0.03 V": 0b0001,  # 1%
        "0.06 V": 0b0010,  # 2%
        "0.12 V": 0b0011,  # 4%
        "0.18 V": 0b0100,  # 6%
        "0.24 V": 0b0101,  # 8%
        "0.30 V": 0b0110,  # 10%
        "0.36 V": 0b0111,  # 12%
        "0.42 V": 0b1000,  # 14%
        "0.48 V": 0b1001,  # 16%
        "0.54 V": 0b1010,  # 18%
        "0.60 V": 0b1011,  # 20%
        "0.66 V": 0b1100,  # 22%
        "0.72 V": 0b1101,  # 24%
    }
    BIAS_DEFAULT = "0.00 V"

    ## MODECN -- Mode control register (address 0x12)
    MODECN_REG = 0x12
    # Shorting FET
    FET_SHORT_REG = MODECN_REG
    FET_SHORT_MASK = 0b10000000
    FET_SHORT_SHIFT = 7
    FET_SHORT_BIN = {
        "Disabled": 0b0,
        "Enabled": 0b1,
    }
    FET_SHORT_DEFAULT = "Disabled"

    # Operating mode
    OP_MODE_REG = MODECN_REG
    OP_MODE_MASK = 0b00000111
    OP_MODE_SHIFT = 0
    OP_MODE_BIN = {
        "Deep sleep": 0b000,
        "2-lead ground": 0b001,
        "Standby": 0b010,
        "3-lead": 0b011,
        "Temp (TIA off)": 0b110,
        "Temp (TIA on)": 0b111,
    }
    OP_MODE_DEFAULT = "Deep sleep"

    # --- Application specific values ---
@dataclass
class LMP91000Settings:
    tia_value = {
        "External": 10,  # Not available
        "2.75 kΩ": 2.75,
        "3.5 kΩ": 3.5,
        "7 kΩ": 7,
        "14 kΩ": 14,
        "35 kΩ": 35,
        "120 kΩ": 120,
        "350 kΩ": 350,
    }
    z_value = {
        "0.6 V": 0.2,
        "1.5 V": 0.5,
        "2.0 V": 0.67,
        "Bypass": 0.0,
    }
    bais_value = {
        "0.00 V": 0.00,
        "0.03 V": 0.01,
        "0.06 V": 0.02,
        "0.12 V": 0.04,
        "0.18 V": 0.06,
        "0.24 V": 0.08,
        "0.30 V": 0.10,
        "0.36 V": 0.12,
        "0.42 V": 0.14,
        "0.48 V": 0.16,
        "0.54 V": 0.18,
        "0.60 V": 0.20,
        "0.66 V": 0.22,
        "0.72 V": 0.24,
    }
    active_value = {
        "Deep sleep": False,
        "2-lead ground": True,
        "Standby": True,
        "3-lead": True,
        "Temp (TIA off)": True,
        "Temp (TIA on)": True,
    }

    # --- Variables ---
    chip: int = 0
    tia_gain: str = LMP91000REGS.TIA_GAIN_DEFAULT
    rload: str = LMP91000REGS.RLOAD_DEFAULT
    ref_source: str = LMP91000REGS.REF_SOURCE_DEFAULT
    int_z: str = LMP91000REGS.INT_Z_DEFAULT
    bias_sign: str = LMP91000REGS.BIAS_SIGN_DEFAULT
    bias: str = LMP91000REGS.BIAS_DEFAULT
    fet_short: str = LMP91000REGS.FET_SHORT_DEFAULT
    op_mode: str = LMP91000REGS.OP_MODE_DEFAULT
    
    active: bool = active_value[LMP91000REGS.OP_MODE_DEFAULT]
    gain: float = tia_value[LMP91000REGS.TIA_GAIN_DEFAULT]
    z: float = z_value[LMP91000REGS.INT_Z_DEFAULT]

    def update(self):
        self.active = self.active_value[self.op_mode]
        self.gain = self.tia_value[self.tia_gain]
        self.z = self.z_value[self.int_z]

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
    ui_settings_changed = pyqtSignal(int, str, str)
    
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
        if key in ["tia_gain", "rload"]:
            setattr(settings, key, value)
            self.write_tiacn_register(settings)
        elif key in ["ref_source", "int_z", "bias_sign", "bias"]:
            setattr(settings, key, value)
            self.write_refcn_register(settings)
        elif key in ["fet_short", "op_mode"]:
            setattr(settings, key, value)
            self.write_modecn_register(settings)
        
        settings.update()
        
    def run(self):
        """Main thread loop: read serial or simulate data at sampling frequency."""
        start_time = time.time()
        while True:
            if not self.running:
                time.sleep(0.01)
                continue
            
            self.current_time = time.time() - start_time
            if self.simulate:
                self.simulate_data()
                time.sleep(self.adc_dt)
            else:
                if self.switch:
                    self.switch_channel()
                self.read_serial()
                time.sleep(self.adc_dt_half)

    def write_register(self, chip, settings, value):
        if not self.serial_port:
            self.log_output.append("Not connected.")
            return
        cmd = "WRITE %d %d %d\n"%(chip, settings, value)
        self.serial_port.write(cmd.encode())
        self.log_output.append(f"Sent: {cmd.strip()}")

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
        self.c1_settings.op_mode = c2_mode
        self.c2_settings.op_mode = c1_mode
        self.c1_settings.update()
        self.c2_settings.update()

        if self.c1_settings.active:
            self.write_modecn_register(self.c2_settings)  # Turn off c2 before turning on c1
            time.sleep(0.01)
            self.write_modecn_register(self.c1_settings)
        elif self.c2_settings.active:
            self.write_modecn_register(self.c1_settings)  # Turn off c1 before turning on c2
            time.sleep(0.01)
            self.write_modecn_register(self.c2_settings)

        # Emit UI update signal
        self.settings_changed.emit()
    
    def simulate_data(self):
        """Generate simulated ADC data at the configured frequency."""
        t = self.current_time
        v1 = 512 + 400 * math.sin(2 * math.pi * 1.0 * t) + 20 * math.sin(t * 10)
        v2 = 512 + 150 * math.cos(2 * math.pi * 0.5 * t) + 10 * math.cos(t * 7)
        self.buffer.append([v1, v2], self.current_time)
        self.active_buffer.append([True, True], self.current_time)

    def write_register(self, chip, settings, value):
        if not self.serial_port:
            self.log_message.emit("Not connected.")
            return
        cmd = "WRITE %d %d %d\n"%(chip, settings, value)
        self.serial_port.write(cmd.encode())
        self.log_message.emit(f"Sent: {cmd.strip()}")
    
    def write_tiacn_register(self, settings):
        tia_gain = LMP91000REGS.TIA_GAIN_BIN[settings.tia_gain] << LMP91000REGS.TIA_GAIN_SHIFT
        rload = LMP91000REGS.RLOAD_BIN[settings.rload] << LMP91000REGS.RLOAD_SHIFT

        tiacn_value = tia_gain | rload
        self.write_register(settings.chip, LMP91000REGS.TIACN_REG, tiacn_value)

    def write_refcn_register(self, settings):        
        ref_source = LMP91000REGS.REF_SOURCE_BIN[settings.ref_source] << LMP91000REGS.REF_SOURCE_SHIFT
        int_z = LMP91000REGS.INT_Z_BIN[settings.int_z] << LMP91000REGS.INT_Z_SHIFT
        bias_sign = LMP91000REGS.BIAS_SIGN_BIN[settings.bias_sign] << LMP91000REGS.BIAS_SIGN_SHIFT
        bias = LMP91000REGS.BIAS_BIN[settings.bias] << LMP91000REGS.BIAS_SHIFT

        refcn_value = ref_source | int_z | bias_sign | bias
        self.write_register(settings.chip, LMP91000REGS.REFCN_REG, refcn_value)

    def write_modecn_register(self, settings):
        fet_short = LMP91000REGS.FET_SHORT_BIN[settings.fet_short] << LMP91000REGS.FET_SHORT_SHIFT
        op_mode = LMP91000REGS.OP_MODE_BIN[settings.op_mode] << LMP91000REGS.OP_MODE_SHIFT
        
        modecn_value = fet_short | op_mode
        self.write_register(settings.chip, LMP91000REGS.MODECN_REG, modecn_value)

    def init_register(self, settings):
        self.write_tiacn_register(settings)
        self.write_refcn_register(settings)
        self.write_modecn_register(settings)

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
                time.sleep(0.05)
                self.init_register(self.c1_settings)
                self.init_register(self.c2_settings)
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
        
        for settings in (self.c1_settings_ui, self.c2_settings_ui):
            chip_setting_layout.addWidget(settings.group)
            settings_layout = QHBoxLayout()
            settings.group.setLayout(settings_layout)

            TIACN_group = QGroupBox("TIA")
            TIACN_layout = QGridLayout()
            TIACN_group.setLayout(TIACN_layout)
            settings_layout.addWidget(TIACN_group)

            settings.tia_gain_combo = QComboBox()
            TIACN_layout.addWidget(QLabel("TIA gain:"), 0, 0)
            TIACN_layout.addWidget(settings.tia_gain_combo, 1, 0)
            settings.tia_gain_combo.addItems(LMP91000REGS.TIA_GAIN_BIN.keys())
            settings.tia_gain_combo.setCurrentText(LMP91000REGS.TIA_GAIN_DEFAULT)
            settings.tia_gain_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "tia_gain"))

            settings.rload_combo = QComboBox()
            TIACN_layout.addWidget(QLabel("R_load:"), 2, 0)
            TIACN_layout.addWidget(settings.rload_combo, 3, 0)
            settings.rload_combo.addItems(LMP91000REGS.RLOAD_BIN.keys())
            settings.rload_combo.setCurrentText(LMP91000REGS.RLOAD_DEFAULT)
            settings.rload_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "rload"))

            REFCN_group = QGroupBox("Reference")
            REFCN_layout = QGridLayout()
            REFCN_group.setLayout(REFCN_layout)
            settings_layout.addWidget(REFCN_group)

            settings.ref_source_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Reference:"), 0, 0)
            REFCN_layout.addWidget(settings.ref_source_combo, 0, 1)
            settings.ref_source_combo.addItems(LMP91000REGS.REF_SOURCE_BIN.keys())
            settings.ref_source_combo.setCurrentText(LMP91000REGS.REF_SOURCE_DEFAULT)
            settings.ref_source_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "ref_source"))

            settings.int_z_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Internal zero:"), 1, 0)
            REFCN_layout.addWidget(settings.int_z_combo, 1, 1)
            settings.int_z_combo.addItems(LMP91000REGS.INT_Z_BIN.keys())
            settings.int_z_combo.setCurrentText(LMP91000REGS.INT_Z_DEFAULT)
            settings.int_z_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "int_z"))

            settings.bias_sign_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Bias polarity:"), 2, 0)
            REFCN_layout.addWidget(settings.bias_sign_combo, 2, 1)
            settings.bias_sign_combo.addItems(LMP91000REGS.BIAS_SIGN_BIN.keys())
            settings.bias_sign_combo.setCurrentText(LMP91000REGS.BIAS_SIGN_DEFAULT)
            settings.bias_sign_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "bias_sign"))

            settings.bias_combo = QComboBox()
            REFCN_layout.addWidget(QLabel("Bias level:"), 3, 0)
            REFCN_layout.addWidget(settings.bias_combo, 3, 1)
            settings.bias_combo.addItems(LMP91000REGS.BIAS_BIN.keys())
            settings.bias_combo.setCurrentText(LMP91000REGS.BIAS_DEFAULT)
            settings.bias_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "bias"))

            MODECN_group = QGroupBox("Mode")
            MODECN_layout = QGridLayout()
            MODECN_group.setLayout(MODECN_layout)
            settings_layout.addWidget(MODECN_group)

            settings.fet_short_combo = QComboBox()
            MODECN_layout.addWidget(QLabel("FET short:"), 0, 0)
            MODECN_layout.addWidget(settings.fet_short_combo, 1, 0)
            settings.fet_short_combo.addItems(LMP91000REGS.FET_SHORT_BIN.keys())
            settings.fet_short_combo.setCurrentText(LMP91000REGS.FET_SHORT_DEFAULT)
            settings.fet_short_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "fet_short"))

            settings.op_mode_combo = QComboBox()
            MODECN_layout.addWidget(QLabel("Operating mode:"), 2, 0)
            MODECN_layout.addWidget(settings.op_mode_combo, 3, 0)
            settings.op_mode_combo.addItems(LMP91000REGS.OP_MODE_BIN.keys())
            settings.op_mode_combo.setCurrentText(LMP91000REGS.OP_MODE_DEFAULT)
            settings.op_mode_combo.currentTextChanged.connect(
                partial(self.worker.ui_settings_changed.emit, settings.chip, "op_mode"))

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
        self.switch_speed_combo.currentTextChanged.connect(self.on_switch_speed_changed)

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

        self.plot1_auto_range = True
        self.plot_item1.getViewBox().sigRangeChangedManually.connect(lambda: self.on_range_changed(1))
        self.plot1_label1 = pg.TextItem("", color="b", anchor=(1, 0))
        self.plot_item1.addItem(self.plot1_label1, ignoreBounds=True)
        self.plot1_label2 = pg.TextItem("", color="r", anchor=(1, 0))
        self.plot_item1.addItem(self.plot1_label2, ignoreBounds=True)
        self.plot1_label2.hide()

        # Bottom plot (channel 2)
        self.plot_item2 = self.plot_widget.addPlot(row=1, col=0)
        self.plot_item2.setLabel("left", "Voltage (V)")
        self.plot_item2.setLabel("bottom", "Time (s)")
        self.plot_item2.showGrid(x=True, y=True)

        self.plot2_auto_range = True
        self.plot_item2.getViewBox().sigRangeChangedManually.connect(lambda: self.on_range_changed(2))
        self.plot2_label2 = pg.TextItem("", color="r", anchor=(1, 0))
        self.plot_item2.addItem(self.plot2_label2, ignoreBounds=True)

        # Disable all mouse interactions on both view boxes (pan/zoom/menus)
        for pi in (self.plot_item1, self.plot_item2):
            vb = pi.getViewBox()
            #vb.setMouseEnabled(x=False, y=True)
            vb.setMenuEnabled(False)

        # plot curves: separate curve per plot item
        self.plot_adc1 = self.plot_item1.plot([], [], pen=pg.mkPen("b", width=2), name='Chip 1')
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
            settings_ui.tia_gain_combo.setCurrentText(settings.tia_gain)
            settings_ui.rload_combo.setCurrentText(settings.rload)
            settings_ui.ref_source_combo.setCurrentText(settings.ref_source)
            settings_ui.int_z_combo.setCurrentText(settings.int_z)
            settings_ui.bias_sign_combo.setCurrentText(settings.bias_sign)
            settings_ui.bias_combo.setCurrentText(settings.bias)
            settings_ui.fet_short_combo.setCurrentText(settings.fet_short)
            settings_ui.op_mode_combo.setCurrentText(settings.op_mode)
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
            self.plot_item2.hide()
            self.plot1_label2.show()
        else:
            self.combine_button.setText("Combine")
            self.plot_item2.show()
            self.plot1_label2.hide()
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
        self.plot1_auto_range = True
        self.plot2_auto_range = True

    def on_range_changed(self, idx):
        if idx == 1:
            self.plot1_auto_range = False
        elif idx == 2:
            self.plot2_auto_range = False

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
        if len(timestamp) == 0:
            return
        tmin = numpy.min(timestamp)
        tmax = numpy.max(timestamp)
        
        if self.combine:
            self.plot_adc1.setData(timestamp, v1)
            self.plot_adc2.setData(timestamp, v2)
            if self.plot1_auto_range:
                self.plot_item1.setXRange(tmin, tmax, padding=0)

            x_right = self.plot_item1.getViewBox().viewRange()[0][1]
            y1_top = self.plot_item1.getViewBox().viewRange()[1][1]
            self.plot1_label1.setText(f"1: {v1[-1]:.2f}")
            self.plot1_label1.setPos(x_right, y1_top)
            self.plot1_label2.setText(f"2: {v2[-1]:.2f}")
            self.plot1_label2.setPos(x_right, y1_top)
        else:
            self.plot_adc1.setData(timestamp, v1)
            self.plot2_adc2.setData(timestamp, v2)
            if self.plot1_auto_range:
                self.plot_item1.setXRange(tmin, tmax, padding=0)
            if self.plot2_auto_range:
                self.plot_item2.setXRange(tmin, tmax, padding=0)

            x_right1 = self.plot_item1.getViewBox().viewRange()[0][1]
            y1_top = self.plot_item1.getViewBox().viewRange()[1][1]
            y2_top = self.plot_item2.getViewBox().viewRange()[1][1]

            self.plot1_label1.setText(f"1: {v1[-1]:.2f}")
            self.plot1_label1.setPos(x_right1, y1_top)
            
            x_right2 = self.plot_item2.getViewBox().viewRange()[0][1]
            self.plot2_label2.setText(f"2: {v2[-1]:.2f}")
            self.plot2_label2.setPos(x_right2, y2_top)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = LMP91000UI()
    gui.show()
    sys.exit(app.exec_())
