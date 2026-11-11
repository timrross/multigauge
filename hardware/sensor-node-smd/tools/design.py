# Single source of truth for the SMD sensor node (rev 3).
# Used by gen_sch.py (schematic), gen_pcb.py (board), gen_jlc.py (assembly files) and the netlist check.
# Same circuit and same ESP32 pins as the rev 2 carrier board, so the firmware is unchanged.
import os, uuid

PROJECT = "sensor-node-smd"
OUT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NS = uuid.UUID("7c2d9a41-0b6e-4f1d-8a35-9e4b6f2d1c03")


def uid(key):
    return str(uuid.uuid5(NS, key))


ROOT_UUID = uid("root-sheet")

R = "Resistor_SMD:R_0603_1608Metric"
C = "Capacitor_SMD:C_0603_1608Metric"
C08 = "Capacitor_SMD:C_0805_2012Metric"
SOIC = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"
TP = "TestPoint:TestPoint_Pad_D1.0mm"
MH = "MountingHole:MountingHole_3.2mm_M3"
SW = "Button_Switch_SMD:SW_SPST_B3U-1000P"

# ref -> (value, symbol lib_id, footprint)
PARTS = {
    "U1": ("ESP32-C3-WROOM-02U-N4", "RF_Module:ESP32-C3-WROOM-02U", "SensorNodeSMD:ESP32-C3-WROOM-02U_Via0.3mm"),
    "U2": ("ADS1115IDGS", "Analog_ADC:ADS1115IDGS", "Package_SO:TSSOP-10_3x3mm_P0.5mm"),
    "U3": ("MAX31855KASA", "Sensor_Temperature:MAX31855KASA", SOIC),
    "U4": ("SN65HVD230", "Interface_CAN_LIN:SN65HVD230", SOIC),
    "U5": ("AP2112K-3.3", "Regulator_Linear:AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5"),
    "J1": ("Micro-Fit 2x8", "Connector_Generic:Conn_02x08_Top_Bottom",
           "Connector_Molex:Molex_Micro-Fit_3.0_43045-1600_2x08_P3.00mm_Horizontal"),
    "J2": ("Thermocouple", "Connector:Screw_Terminal_01x02",
           "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal"),
    "J3": ("USB-C", "Connector:USB_C_Receptacle_USB2.0_16P", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12"),
    "L1": ("47uH", "Device:L", "Inductor_SMD:L_Changjiang_FNR6045S"),
    "C1": ("100uF 25V", "Device:C_Polarized", "Capacitor_SMD:CP_Elec_6.3x7.7"),
    "C2": ("100nF", "Device:C", C),
    "C3": ("100nF", "Device:C", C), "C4": ("100nF", "Device:C", C),
    "C5": ("100nF", "Device:C", C), "C6": ("100nF", "Device:C", C),
    "C7": ("10uF", "Device:C", C08), "C8": ("10uF", "Device:C", C08), "C9": ("10uF", "Device:C", C08),
    "C10": ("100nF", "Device:C", C), "C11": ("1uF", "Device:C", C), "C12": ("100nF", "Device:C", C),
    "C13": ("100nF", "Device:C", C), "C14": ("10nF", "Device:C", C), "C15": ("100nF", "Device:C", C),
    "D1": ("B5819W", "Device:D_Schottky", "Diode_SMD:D_SOD-123"),
    "D2": ("B5819W", "Device:D_Schottky", "Diode_SMD:D_SOD-123"),
    "D3": ("Green", "Device:LED", "LED_SMD:LED_0603_1608Metric"),
    "R1": ("10k 1%", "Device:R", R), "R2": ("15k 1%", "Device:R", R),
    "R3": ("10k 1%", "Device:R", R), "R4": ("15k 1%", "Device:R", R),
    "R5": ("330R 1%", "Device:R", R), "R6": ("2k2 1%", "Device:R", R),
    "R7": ("10k 1%", "Device:R", R), "R8": ("10k 1%", "Device:R", R),
    "R9": ("10k 1%", "Device:R", R), "R10": ("10k 1%", "Device:R", R),
    "R11": ("10k 1%", "Device:R", R), "R12": ("10k 1%", "Device:R", R),
    "R13": ("5k1", "Device:R", R), "R14": ("5k1", "Device:R", R),
    "R15": ("4k7", "Device:R", R), "R16": ("4k7", "Device:R", R),
    "R17": ("120R", "Device:R", R), "R18": ("1k", "Device:R", R),
    "SW1": ("RESET", "Switch:SW_Push", SW), "SW2": ("BOOT", "Switch:SW_Push", SW),
    "JP1": ("CAN term", "Jumper:SolderJumper_2_Bridged", "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm"),
    "TP1": ("+5V", "Connector:TestPoint", TP), "TP2": ("+3V3", "Connector:TestPoint", TP),
    "TP3": ("GND", "Connector:TestPoint", TP), "TP4": ("OIL_P", "Connector:TestPoint", TP),
    "TP5": ("BOOST", "Connector:TestPoint", TP), "TP6": ("OIL_T", "Connector:TestPoint", TP),
    "TP7": ("IAT", "Connector:TestPoint", TP), "TP8": ("SDA", "Connector:TestPoint", TP),
    "TP9": ("SCL", "Connector:TestPoint", TP),
    "H1": ("M3", "Mechanical:MountingHole", MH), "H2": ("M3", "Mechanical:MountingHole", MH),
    "H3": ("M3", "Mechanical:MountingHole", MH), "H4": ("M3", "Mechanical:MountingHole", MH),
}

# J1 pin map. Pin n and pin n+8 share a column, so each sensor's wires sit together in the plug.
J1_MAP = {
    "8": ("5V_IN", "+5V in from the buck converter"), "16": ("GND", "GND in"),
    "7": ("CANH", "CAN high"), "15": ("CANL", "CAN low"),
    "6": ("+5V", "+5V to oil pressure sensor"), "14": ("GND", "GND oil pressure sensor"),
    "5": ("OIL_P_RAW", "Oil pressure signal"), "13": (None, "Spare (not connected)"),
    "4": ("+5V", "+5V to boost sensor"), "12": ("GND", "GND boost sensor"),
    "3": ("BOOST_RAW", "Boost signal"), "11": (None, "Spare (not connected)"),
    "2": ("OIL_T_RAW", "Oil temp NTC"), "10": ("GND", "Oil temp NTC return"),
    "1": ("IAT_RAW", "Intake temp NTC"), "9": ("GND", "Intake temp NTC return"),
}

# net -> [(ref, pin)]
NETS = {
    "5V_IN": [("L1", "1")],
    "+5V": [("L1", "2"), ("C1", "1"), ("C2", "1"), ("D1", "2"), ("TP1", "1")],
    "VBUS": [("J3", "A4"), ("J3", "A9"), ("J3", "B4"), ("J3", "B9"), ("D2", "2")],
    "VSYS": [("D1", "1"), ("D2", "1"), ("U5", "1"), ("U5", "3"), ("C7", "1"), ("R18", "1")],
    "+3V3": [("U5", "5"), ("C8", "1"), ("C9", "1"), ("C10", "1"), ("U1", "1"), ("R9", "1"), ("R10", "1"),
             ("R11", "1"), ("R12", "1"), ("U2", "8"), ("C12", "1"), ("R15", "1"), ("R16", "1"), ("U3", "4"),
             ("C13", "1"), ("U4", "3"), ("C15", "1"), ("R5", "1"), ("R6", "1"), ("TP2", "1")],
    "GND": [("C1", "2"), ("C2", "2"), ("C7", "2"), ("C8", "2"), ("C9", "2"), ("C10", "2"), ("C11", "2"),
            ("U1", "9"), ("U1", "19"), ("U5", "2"), ("U2", "3"), ("U2", "1"), ("C12", "2"), ("U3", "1"),
            ("C13", "2"), ("U4", "2"), ("U4", "8"), ("C15", "2"), ("R2", "2"), ("R4", "2"),
            ("C3", "2"), ("C4", "2"), ("C5", "2"), ("C6", "2"), ("R13", "2"), ("R14", "2"),
            ("J3", "A1"), ("J3", "A12"), ("J3", "B1"), ("J3", "B12"), ("J3", "SH"),
            ("SW1", "2"), ("SW2", "2"), ("D3", "1"), ("TP3", "1")],
    "EN": [("U1", "2"), ("R9", "2"), ("C11", "1"), ("SW1", "1")],
    "BOOT": [("U1", "8"), ("R10", "2"), ("SW2", "1")],
    "IO8": [("U1", "7"), ("R11", "2")],
    "IO2": [("U1", "16"), ("R12", "2")],
    "USB_DP": [("U1", "14"), ("J3", "A6"), ("J3", "B6")],
    "USB_DM": [("U1", "13"), ("J3", "A7"), ("J3", "B7")],
    "CC1": [("J3", "A5"), ("R13", "1")],
    "CC2": [("J3", "B5"), ("R14", "1")],
    "SDA": [("U1", "3"), ("U2", "9"), ("R15", "2"), ("TP8", "1")],
    "SCL": [("U1", "4"), ("U2", "10"), ("R16", "2"), ("TP9", "1")],
    "TC_CLK": [("U1", "5"), ("U3", "5")],
    "TC_CS": [("U1", "6"), ("U3", "6")],
    "TC_DO": [("U1", "15"), ("U3", "7")],
    "TC_P": [("J2", "2"), ("U3", "3"), ("C14", "1")],
    "TC_N": [("J2", "1"), ("U3", "2"), ("C14", "2")],
    "CAN_TX": [("U1", "18"), ("U4", "1")],
    "CAN_RX": [("U1", "17"), ("U4", "4")],
    "CANH": [("U4", "7"), ("R17", "1")],
    "CANL": [("U4", "6"), ("JP1", "2")],
    "CAN_TERM": [("R17", "2"), ("JP1", "1")],
    "OIL_P_RAW": [("R1", "1")],
    "OIL_P": [("R1", "2"), ("R2", "1"), ("C3", "1"), ("U2", "4"), ("TP4", "1")],
    "BOOST_RAW": [("R3", "1")],
    "BOOST": [("R3", "2"), ("R4", "1"), ("C4", "1"), ("U2", "5"), ("TP5", "1")],
    "OIL_T_RAW": [("R5", "2"), ("R7", "1")],
    "OIL_T": [("R7", "2"), ("C5", "1"), ("U2", "6"), ("TP6", "1")],
    "IAT_RAW": [("R6", "2"), ("R8", "1")],
    "IAT": [("R8", "2"), ("C6", "1"), ("U2", "7"), ("TP7", "1")],
    "LED_A": [("R18", "2"), ("D3", "2")],
}
NO_CONNECT = [("U1", "10"), ("U1", "11"), ("U1", "12"), ("U2", "2"), ("U3", "8"), ("U4", "5"), ("U5", "4"),
              ("J3", "A8"), ("J3", "B8")]
for _pin, (_net, _note) in J1_MAP.items():
    if _net is None: NO_CONNECT.append(("J1", _pin))
    else: NETS[_net].append(("J1", _pin))

POWER_NETS = ["5V_IN", "+5V", "VBUS", "VSYS", "+3V3"]

PIN_NET = {}
for _n, _pins in NETS.items():
    for _rp in _pins:
        assert _rp not in PIN_NET, _rp
        PIN_NET[_rp] = _n

# ---- JLCPCB assembly data: ref -> (LCSC part number, description for the BOM). None = not fitted by JLCPCB.
_res = {"10k 1%": "C25804", "15k 1%": "C22809", "330R 1%": "C23138", "2k2 1%": "C4190",
        "5k1": "C23186", "4k7": "C23162", "120R": "C22787", "1k": "C21190"}
_cap = {"100nF": "C14663", "1uF": "C15849", "10nF": "C57112", "10uF": "C15850"}
LCSC = {}
for _r, (_v, _s, _f) in PARTS.items():
    if _r[0] == "R": LCSC[_r] = _res[_v]
    elif _r[0] == "C" and _r != "C1": LCSC[_r] = _cap[_v]
# Every number below was checked against JLCPCB's live catalogue on 2026-11-11 (part identity and stock).
# L1 is the Changjiang FNR6045S470MT: 0.26 ohm max, 1.3A. C1 is 100uF 25V, 6.3 x 7.7mm, 105C.
# D3 is an emerald green LED with a forward voltage near 3.2V, which is why it runs from VSYS and not +3V3.
LCSC.update({"U1": "C2926676", "U2": "C37593", "U3": "C52028", "U4": "C12084", "U5": "C51118",
             "J3": "C165948", "L1": "C168087", "C1": "C3338", "D1": "C8598", "D2": "C8598", "D3": "C2986059",
             "SW1": "C231329", "SW2": "C231329"})
HAND_FIT = {"J1": "Molex Micro-Fit 3.0 right-angle header, 2x8, part 43045-1600",
            "J2": "2-way 5.0mm screw terminal (from the parts box)"}
