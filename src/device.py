"""
    Project: BLISS
    Application: BLISS Sensor Board User Interface
    File: src/device.py
    Description: Device controls.
    Author: Fang Zihang (Dr.)
    Email: zh.fang@nus.edu.sg
    Affiliation: National University of Singapore
"""
import numpy as np
import os
import sys
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except AttributeError:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

# --- CONSTANTS ---
# * DO NOT CHANGE *
DEBUG_MODE = False

ADC_RESOLUTION = 10
ADC_N = (1 << ADC_RESOLUTION)
LMP91000_VREF = 3.0
ADC_VREF = 3.3
ADC_VSTEP = ADC_VREF / ADC_N

LMP91000_TEMP_LUT = np.loadtxt(resource_path("lmp91000_temp_lut.csv"), delimiter=",", dtype=float)

# --- LMP91000 constants ---
class LMP91000:
    reg_keys = [
        "tia_gain",
        "rload",
        "ref_source",
        "int_z",
        "bias_sign",
        "bias",
        "fet_short",
        "op_mode",
    ]
    active_keys = [
        "tia_gain",
        "int_z",
        "bias_sign",
        "bias",
        "op_mode",
    ]

    ## TIACN -- TIA control register (address 0x10)
    tiacn_reg = 0x10

    # Transimpedance amplifier gain settings 
    tia_gain_shift = 2
    tia_gain_bin = [
        0b000,
        0b001,
        0b010,
        0b011,
        0b100,
        0b101,
        0b110,
        0b111,
    ]
    tia_gain_text = [
        "External",
        "2.75 kΩ",
        "3.5 kΩ",
        "7 kΩ",
        "14 kΩ",
        "35 kΩ",
        "120 kΩ",
        "350 kΩ",
    ]
    tia_gain_label = "Gain"
    tia_gain_default = 6    # index for "120 kΩ"

    # Load resistance settings
    rload_shift = 0
    rload_bin = [
        0b00,
        0b01,
        0b10,
        0b11,   
    ]
    rload_text = [
        "10 Ω",
        "33 Ω",
        "50 Ω",
        "100 Ω",
    ]
    rload_label = "Rload"
    rload_default = 3   # index for "100 Ω"

    ## REFCN -- Reference control register (address 0x11)
    refcn_reg = 0x11

    # Reference voltage source selection
    ref_source_shift = 7
    ref_source_bin = [
        0b0,
        0b1,
    ]
    ref_source_text = [
        "Internal",
        "External",
    ]
    ref_source_label = "Source"
    ref_source_default = 1  # index for "External"

    # Internal zero selection, percentage of the source reference
    int_z_shift = 5
    int_z_bin = [
        0b00,
        0b01,
        0b10,
        0b11,
    ]
    _int_z_text = [
        "20%",
        "50%",
        "67%",
        "Bypass",
    ]
    int_z_label = "Vref"
    int_z_default = 1   # index for "50%"

    # Bias polarity selection, (V_WE - V_RE) sign
    bias_sign_shift = 4
    bias_sign_bin = [
        0b0,
        0b1,
    ]
    bias_sign_text = [
        "Negative WE<RE",
        "Positive WE>RE",
    ]
    bias_sign_label = "Bias ±"
    bias_sign_default = 1   # index for "Positive WE>RE"

    # Bias voltage level, percentage of the source reference
    bias_shift = 0
    bias_bin = [
        0b0000,
        0b0001,
        0b0010,
        0b0011,
        0b0100,
        0b0101,
        0b0110,
        0b0111,
        0b1000,
        0b1001,
        0b1010,
        0b1011,
        0b1100,
        0b1101,
    ]
    _bias_text = [
        "0%",
        "1%",
        "2%",
        "4%",
        "6%",
        "8%",
        "10%",
        "12%",
        "14%",
        "16%",
        "18%",
        "20%",
        "22%",
        "24%",
    ]
    bias_label = "Vbias"
    bias_default = 0    # index for "0%"

    ## MODECN -- Mode control register (address 0x12)
    modecn_reg = 0x12

    # Shorting FET
    fet_short_shift = 7
    fet_short_bin = [
        0b0,
        0b1,
    ]
    fet_short_text = [
        "Disabled",
        "Enabled",
    ]
    fet_short_label = "FET"
    fet_short_default = 0   # index for "Disabled"

    # Operating mode
    op_mode_shift = 0
    op_mode_bin = [
        0b000,
        0b001,
        0b010,
        0b011,
        0b110,
        0b111,
    ]
    op_mode_text = [
        "Deep sleep",
        "2-lead ground",
        "Standby",
        "3-lead",
        "Temp (TIA off)",
        "Temp (TIA on)",
    ]
    op_mode_label = "Mode"
    op_mode_default = 0 # index for "Deep sleep"

    # --- Application specific values (kept lowercase) ---
    gain_value = [
        10,  # Not available
        2.75,
        3.5,
        7,
        14,
        35,
        120,
        350,
    ]
    z_value = [
        0.2,
        0.5,
        0.67,
        0.0,
    ]
    active_value = [
        False,
        True,
        True,
        True,
        True,
        True,
    ]
    volt_lut = LMP91000_TEMP_LUT[:, 0]
    temp_lut = LMP91000_TEMP_LUT[:, 1]

    def __init__(self, id, vref=LMP91000_VREF):
        self.id = id
        self.vref = vref

        self.tia_gain = self.tia_gain_default
        self.rload = self.rload_default
        self.ref_source = self.ref_source_default
        self.int_z = self.int_z_default
        self.bias_sign = self.bias_sign_default
        self.bias = self.bias_default
        self.fet_short = self.fet_short_default
        self.op_mode = self.op_mode_default

        self.active = self.active_value[self.op_mode]
        self.gain_val = self.gain_value[self.tia_gain]
        self.z_val = self.z_value[self.int_z]

        # Update int_z text
        self.int_z_text = self._int_z_text.copy()
        for i in range(len(self.int_z_text) - 1):
            precent_str = self.int_z_text[i]
            precent_float = float(precent_str.strip("%")) / 100
            value = self.vref * precent_float
            self.int_z_text[i] = f"{value:.2f} V"
        
        # Update bias text
        self.bias_text = self._bias_text.copy()
        for i in range(len(self.bias_text)):
            precent_str = self.bias_text[i]
            precent_float = float(precent_str.strip("%")) / 100
            value = self.vref * precent_float
            self.bias_text[i] = f"{value:.2f} V"

    def update_tiacn(self):
        tia_gain = self.tia_gain_bin[self.tia_gain] << self.tia_gain_shift
        rload = self.rload_bin[self.rload] << self.rload_shift
        tiacn = tia_gain | rload
        
        self.gain_val = self.gain_value[self.tia_gain]
        return "WRITE %d %d %d\n"%(self.id, self.tiacn_reg, tiacn)

    def update_refcn(self):
        ref_source = self.ref_source_bin[self.ref_source] << self.ref_source_shift
        int_z = self.int_z_bin[self.int_z] << self.int_z_shift
        bias_sign = self.bias_sign_bin[self.bias_sign] << self.bias_sign_shift
        bias = self.bias_bin[self.bias] << self.bias_shift
        refcn = ref_source | int_z | bias_sign | bias
        
        self.z_val = self.z_value[self.int_z]
        return  "WRITE %d %d %d\n"%(self.id, self.refcn_reg, refcn)

    def update_modecn(self):
        fet_short = self.fet_short_bin[self.fet_short] << self.fet_short_shift
        op_mode = self.op_mode_bin[self.op_mode] << self.op_mode_shift
        modecn = fet_short | op_mode
        
        self.active = self.active_value[self.op_mode]
        return  "WRITE %d %d %d\n"%(self.id, self.modecn_reg, modecn)

    def update(self, key, index):
        """Update a single setting and return (id, register, value) tuple to write."""
        if key not in self.reg_keys:
            return None
        setattr(self, key, index)

        # return register write tuple
        if key in ("tia_gain", "rload"):
            return self.update_tiacn()
        if key in ("ref_source", "int_z", "bias_sign", "bias"):
            return self.update_refcn()
        if key in ("fet_short", "op_mode"):
            return self.update_modecn()

    def update_all(self):
        """Return list of register write commands for all config registers."""
        return self.update_tiacn() + self.update_refcn() + self.update_modecn()
    
    def get_yrange(self):
        if DEBUG_MODE:
            return 0x000, 0x3FF
        if self.op_mode == 0:
            return 0, self.vref
        if self.op_mode in [4, 5]:
            return 20, 50
        z_voltage = self.vref * self.z_val
        ymin = (-z_voltage) / self.gain_val * 1e3  # uA
        ymax = (self.vref - z_voltage) / self.gain_val * 1e3  # uA
        return ymin, ymax
    
    def get_ylabel(self):
        if DEBUG_MODE:
            return "DEBUG MODE"
        if self.op_mode == 0:
            return "Voltage (V)"
        if self.op_mode in [4, 5]:  # Temperature sensor
            return "Temp (°C)"
        return "Current (uA)"
    
    def formula(self, x):
        if DEBUG_MODE:
            return x
        if self.op_mode == 0:
            return x * ADC_VSTEP
        if self.op_mode in [4, 5]:  # Temperature sensor
            return np.interp(x * ADC_VSTEP, self.volt_lut, self.temp_lut)
        return ((x * ADC_VSTEP - self.vref * self.z_val) / self.gain_val * 1e3)
    
class PHSensor:
    reg_keys = [
        "gain",
    ]
    active_keys = [
        "gain",
    ]
    gain_text = [
        "1x",
        "10x",
        "40x",
        # "200x",
    ]
    gain_label = "Gain"
    gain_default = 0    # 1x

    gain_value = [
        1,
        10,
        40,
        200,
    ]

    def __init__(self, id):
        self.id = id

        self.gain = self.gain_default
        
        self.update_gain()

    def update_gain(self):
        self.gain_val = self.gain_value[self.gain]
        return "WRITE %d %d %d\n"%(self.id, 0, self.gain)

    def update(self, key, index):
        if key not in self.reg_keys:
            return None
        setattr(self, key, index)

        return self.update_gain()

    def update_all(self):
        return self.update_gain()

    def get_yrange(self):
        if DEBUG_MODE:
            return 0x000, 0x3FF
        return self.formula(0x200), self.formula(0x1FF)

    def get_ylabel(self):
        if DEBUG_MODE:
            return "DEBUG MODE"
        return "Voltage (V)"

    def formula(self, x):
        if DEBUG_MODE:
            return x
        # 0x200 to 0x3FF, -512 to -1
        # 0x000 to 0x1FF, 0 to 511
        if x > 0x1FF:
            x = x - 0x400
        return (x * ADC_VSTEP * 2) / self.gain_val