class LMP91000Settings:
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

    ## TIACN -- TIA control register (address 0x10)
    tiacn_reg = 0x10

    # Transimpedance amplifier gain settings 
    tia_gain_mask = 0b00011100
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
    tia_gain_default = 6    # index for "120 kΩ"

    # Load resistance settings
    rload_mask = 0b00000011
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
    rload_default = 3   # index for "100 Ω"

    ## REFCN -- Reference control register (address 0x11)
    refcn_reg = 0x11

    # Reference voltage source selection
    ref_source_mask = 0b10000000
    ref_source_shift = 7
    ref_source_bin = [
        0b0,
        0b1,
    ]
    ref_source_text = [
        "Internal",
        "External",
    ]
    ref_source_default = 1  # index for "External"

    # Internal zero selection, percentage of the source reference
    int_z_mask = 0b01100000
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
    int_z_default = 1   # index for "50%"

    # Bias polarity selection, (V_WE - V_RE) sign
    bias_sign_mask = 0b00010000
    bias_sign_shift = 4
    bias_sign_bin = [
        0b0,
        0b1,
    ]
    bias_sign_text = [
        "Negative WE<RE",
        "Positive WE>RE",
    ]
    bias_sign_default = 1   # index for "Positive WE>RE"

    # Bias voltage level, percentage of the source reference
    bias_mask = 0b00001111
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
    bias_default = 0    # index for "0%"

    ## MODECN -- Mode control register (address 0x12)
    modecn_reg = 0x12

    # Shorting FET
    fet_short_mask = 0b10000000
    fet_short_shift = 7
    fet_short_bin = [
        0b0,
        0b1,
    ]
    fet_short_text = [
        "Disabled",
        "Enabled",
    ]
    fet_short_default = 0   # index for "Disabled"

    # Operating mode
    op_mode_mask = 0b00000111
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

    def __init__(self, chip, vref=LMP91000_VREF):
        self.chip = chip
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
        self.gain = self.gain_value[self.tia_gain]
        self.z = self.z_value[self.int_z]

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

    def get_all(self, key):
        if key in LMP91000Settings.reg_keys:
            attr = getattr(self, key + "_text")
            return list(attr)
        else:
            return [""]

    def get_current(self, key):
        if key in LMP91000Settings.reg_keys:
            attr = getattr(self, key + "_text")
            index = getattr(self, key)
            return attr[index]
        else:
            return ""

    def update_tiacn(self):
        tia_gain_val = self.tia_gain_bin[self.tia_gain] << self.tia_gain_shift
        rload_val = self.rload_bin[self.rload] << self.rload_shift
        tiacn_val = tia_gain_val | rload_val
        
        self.gain = self.gain_value[self.tia_gain]
        return "WRITE %d %d %d\n"%(self.chip, self.tiacn_reg, tiacn_val)

    def update_refcn(self):
        ref_source_val = self.ref_source_bin[self.ref_source] << self.ref_source_shift
        int_z_val = self.int_z_bin[self.int_z] << self.int_z_shift
        bias_sign_val = self.bias_sign_bin[self.bias_sign] << self.bias_sign_shift
        bias_val = self.bias_bin[self.bias] << self.bias_shift
        refcn_val = ref_source_val | int_z_val | bias_sign_val | bias_val
        
        self.z = self.z_value[self.int_z]
        return  "WRITE %d %d %d\n"%(self.chip, self.refcn_reg, refcn_val)

    def update_modecn(self):
        fet_short_val = self.fet_short_bin[self.fet_short] << self.fet_short_shift
        op_mode_val = self.op_mode_bin[self.op_mode] << self.op_mode_shift
        modecn_val = fet_short_val | op_mode_val
        
        self.active = self.active_value[self.op_mode]
        return  "WRITE %d %d %d\n"%(self.chip, self.modecn_reg, modecn_val)

    def update(self, key, index):
        """Update a single setting and return (chip, register, value) tuple to write."""
        if key not in LMP91000Settings.reg_keys:
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
