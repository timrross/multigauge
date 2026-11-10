# Single source of truth for the sensor node carrier board.
# Used by gen_sch.py (schematic) and gen_pcb.py (board), and by the netlist check.
import uuid

PROJECT = "sensor-node-carrier"
import os
OUT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NS = uuid.UUID("3b0e7f5e-6a53-4a8e-9d3f-5d2c1c7a9e10")


def uid(key):
    return str(uuid.uuid5(NS, key))


ROOT_UUID = uid("root-sheet")

R_FP = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal"
C_FP = "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm"
TP_FP = "TestPoint:TestPoint_THTPad_D2.0mm_Drill1.0mm"
MH_FP = "MountingHole:MountingHole_3.2mm_M3"

# ref -> (value, symbol lib_id, footprint)
PARTS = {
    "U1": ("ESP32-C3-Zero", "SensorNode:ESP32-C3-Zero", "SensorNode:Module_Waveshare_ESP32-C3-Zero"),
    "U2": ("ADS1115 module", "SensorNode:ADS1115_Module", "SensorNode:Module_ADS1115_1x10"),
    "U3": ("MAX31855 module", "SensorNode:MAX31855_Module", "SensorNode:Module_MAX31855_1x06"),
    "U4": ("SN65HVD230 CAN module", "SensorNode:CAN_Module", "SensorNode:Module_SN65HVD230_1x04"),
    "J1": ("Loom 12-way", "Connector:Screw_Terminal_01x12",
           "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-12-5.08_1x12_P5.08mm_Horizontal"),
    "J2": ("5V in", "Connector:Screw_Terminal_01x02",
           "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal"),
    "J3": ("Spare loom pins", "Connector_Generic:Conn_01x02",
           "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"),
    "J4": ("I2C expansion", "Connector_Generic:Conn_01x04",
           "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"),
    "L1": ("47uH <0.5R", "Device:L", "Inductor_THT:L_Axial_L12.0mm_D5.0mm_P15.24mm_Horizontal_Fastron_MISC"),
    "C1": ("100uF 25V", "Device:C_Polarized", "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm"),
    "C2": ("100nF", "Device:C", C_FP),
    "C3": ("100nF", "Device:C", C_FP),
    "C4": ("100nF", "Device:C", C_FP),
    "C5": ("100nF", "Device:C", C_FP),
    "C6": ("100nF", "Device:C", C_FP),
    "D1": ("1N5817", "Device:D_Schottky", "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal"),
    "R1": ("10k 1%", "Device:R", R_FP),
    "R2": ("15k 1%", "Device:R", R_FP),
    "R3": ("10k 1%", "Device:R", R_FP),
    "R4": ("15k 1%", "Device:R", R_FP),
    "R5": ("330R 1%", "Device:R", R_FP),
    "R6": ("2k2 1%", "Device:R", R_FP),
    "R7": ("10k 1%", "Device:R", R_FP),
    "R8": ("10k 1%", "Device:R", R_FP),
    "TP1": ("+5V", "Connector:TestPoint", TP_FP),
    "TP2": ("+3V3", "Connector:TestPoint", TP_FP),
    "TP3": ("GND", "Connector:TestPoint", TP_FP),
    "TP4": ("OIL_P", "Connector:TestPoint", TP_FP),
    "TP5": ("BOOST", "Connector:TestPoint", TP_FP),
    "TP6": ("OIL_T", "Connector:TestPoint", TP_FP),
    "TP7": ("IAT", "Connector:TestPoint", TP_FP),
    "H1": ("M3", "Mechanical:MountingHole", MH_FP),
    "H2": ("M3", "Mechanical:MountingHole", MH_FP),
    "H3": ("M3", "Mechanical:MountingHole", MH_FP),
    "H4": ("M3", "Mechanical:MountingHole", MH_FP),
}

# Module pin definitions: (number, name, electrical type)
U1_LEFT = [("1", "5V", "power_in"), ("2", "GND", "power_in"), ("3", "3V3", "power_out"),
           ("4", "GP0", "bidirectional"), ("5", "GP1", "bidirectional"), ("6", "GP2", "bidirectional"),
           ("7", "GP3", "bidirectional"), ("8", "GP4", "bidirectional"), ("9", "GP5", "bidirectional")]
U1_RIGHT = [("10", "GP21/TX", "bidirectional"), ("11", "GP20/RX", "bidirectional"),
            ("12", "GP19", "bidirectional"), ("13", "GP18", "bidirectional"), ("14", "GP10", "bidirectional"),
            ("15", "GP9", "bidirectional"), ("16", "GP8", "bidirectional"), ("17", "GP7", "bidirectional"),
            ("18", "GP6", "bidirectional")]
U2_PINS = [("1", "VDD", "power_in"), ("2", "GND", "power_in"), ("3", "SCL", "input"),
           ("4", "SDA", "bidirectional"), ("5", "ADDR", "input"), ("6", "ALRT", "open_collector"),
           ("7", "A0", "passive"), ("8", "A1", "passive"), ("9", "A2", "passive"), ("10", "A3", "passive")]
U3_PINS = [("1", "Vin", "power_in"), ("2", "3Vo", "power_out"), ("3", "GND", "power_in"),
           ("4", "DO", "output"), ("5", "CS", "input"), ("6", "CLK", "input")]
U4_PINS = [("1", "3V3", "power_in"), ("2", "GND", "power_in"), ("3", "CAN_RX", "output"),
           ("4", "CAN_TX", "input")]

# net -> [(ref, pin)]
NETS = {
    "5V_IN": [("J2", "1"), ("L1", "1")],
    "+5V": [("L1", "2"), ("C1", "1"), ("C2", "1"), ("D1", "2"), ("U3", "1"), ("J1", "1"), ("J1", "9"),
            ("TP1", "1")],
    "+5V_MCU": [("D1", "1"), ("U1", "1")],
    "+3V3": [("U1", "3"), ("U2", "1"), ("U4", "1"), ("R5", "1"), ("R6", "1"), ("J4", "1"), ("TP2", "1")],
    "GND": [("J2", "2"), ("C1", "2"), ("C2", "2"), ("U1", "2"), ("U2", "2"), ("U2", "5"), ("U3", "3"),
            ("U4", "2"), ("J1", "2"), ("J1", "6"), ("J1", "8"), ("J1", "10"), ("R2", "2"), ("R4", "2"),
            ("C3", "2"), ("C4", "2"), ("C5", "2"), ("C6", "2"), ("J4", "2"), ("TP3", "1")],
    "OIL_P_RAW": [("J1", "3"), ("R1", "1")],
    "OIL_P": [("R1", "2"), ("R2", "1"), ("C3", "1"), ("U2", "7"), ("TP4", "1")],
    "BOOST_RAW": [("J1", "4"), ("R3", "1")],
    "BOOST": [("R3", "2"), ("R4", "1"), ("C4", "1"), ("U2", "8"), ("TP5", "1")],
    "OIL_T_RAW": [("J1", "5"), ("R5", "2"), ("R7", "1")],
    "OIL_T": [("R7", "2"), ("C5", "1"), ("U2", "9"), ("TP6", "1")],
    "IAT_RAW": [("J1", "7"), ("R6", "2"), ("R8", "1")],
    "IAT": [("R8", "2"), ("C6", "1"), ("U2", "10"), ("TP7", "1")],
    "SDA": [("U1", "8"), ("U2", "4"), ("J4", "3")],
    "SCL": [("U1", "9"), ("U2", "3"), ("J4", "4")],
    "TC_DO": [("U1", "7"), ("U3", "4")],
    "TC_CS": [("U1", "17"), ("U3", "5")],
    "TC_CLK": [("U1", "18"), ("U3", "6")],
    "CAN_TX": [("U1", "4"), ("U4", "4")],
    "CAN_RX": [("U1", "5"), ("U4", "3")],
    "SPARE1": [("J1", "11"), ("J3", "1")],
    "SPARE2": [("J1", "12"), ("J3", "2")],
}

NO_CONNECT = [("U1", p) for p in ("6", "10", "11", "12", "13", "14", "15", "16")] + [("U2", "6"), ("U3", "2")]

POWER_NETS = ["5V_IN", "+5V", "+5V_MCU", "+3V3"]

PIN_NET = {}
for _n, _pins in NETS.items():
    for _rp in _pins:
        assert _rp not in PIN_NET, _rp
        PIN_NET[_rp] = _n
