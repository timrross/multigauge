# Generates SensorNode.pretty (module socket footprints) + fp-lib-table
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import *

def f(v):
    s = ("%.3f" % v).rstrip("0").rstrip("."); return "0" if s in ("-0", "") else s

def line(x1, y1, x2, y2, layer, w=0.12):
    return '\t(fp_line (start %s %s) (end %s %s) (stroke (width %s) (type solid)) (layer "%s"))' % (f(x1), f(y1), f(x2), f(y2), f(w), layer)

def rect(x1, y1, x2, y2, layer, w=0.12):
    return '\t(fp_rect (start %s %s) (end %s %s) (stroke (width %s) (type solid)) (fill no) (layer "%s"))' % (f(x1), f(y1), f(x2), f(y2), f(w), layer)

def txt(s, x, y, ang, layer, size=0.8, just=None):
    return '\t(fp_text user "%s" (at %s %s %d) (layer "%s") (effects (font (size %s %s) (thickness %s))%s))' % (
        s, f(x), f(y), ang, layer, f(size), f(size), f(size * 0.15), (" (justify %s)" % just) if just else "")

def pad(num, x, y):
    return '\t(pad "%s" thru_hole %s (at %s %s) (size 1.7 1.7) (drill 1) (layers "*.Cu" "*.Mask"))' % (
        num, "rect" if num == "1" else "circle", f(x), f(y))

def footprint(name, descr, value, body, refpos, items):
    x1, y1, x2, y2 = body
    out = ['(footprint "%s"' % name, '\t(version 20241229)', '\t(generator "pcbnew")', '\t(generator_version "9.0")',
           '\t(layer "F.Cu")', '\t(descr "%s")' % descr, '\t(tags "module socket")',
           '\t(property "Reference" "REF**" (at %s %s 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))' % (f(refpos[0]), f(refpos[1])),
           '\t(property "Value" "%s" (at %s %s 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))' % (value, f((x1 + x2) / 2), f((y1 + y2) / 2 + 2)),
           '\t(attr through_hole)',
           rect(x1, y1, x2, y2, "F.SilkS"), rect(x1, y1, x2, y2, "F.Fab", 0.1),
           rect(x1 - 0.25, y1 - 0.25, x2 + 0.25, y2 + 0.25, "F.CrtYd", 0.05)] + items + [')', '']
    os.makedirs(OUT + "/SensorNode.pretty", exist_ok=True)
    open("%s/SensorNode.pretty/%s.kicad_mod" % (OUT, name), "w").write("\n".join(out))

# ---- ESP32-C3-Zero: 2 x 9, rows 15.24 apart, USB end at the top (pin 1 / pin 10 end). Board 18 x 23.5
it = []
for i, (num, nm, _) in enumerate(U1_LEFT):
    it += [pad(num, 0, i * 2.54), txt(nm, -1.8, i * 2.54, 0, "F.SilkS", just="right")]
for i, (num, nm, _) in enumerate(U1_RIGHT):
    it += [pad(num, 15.24, i * 2.54), txt(nm.split("/")[0], 17.04, i * 2.54, 0, "F.SilkS", just="left")]
it += [rect(3.12, -1.8, 12.12, 4.3, "F.SilkS"), txt("USB", 7.62, 1.2, 0, "F.SilkS", 1.0),
       rect(3.12, -3.0, 12.12, 4.3, "F.Fab", 0.1)]
footprint("Module_Waveshare_ESP32-C3-Zero", "Waveshare ESP32-C3-Zero, two 1x9 rows 15.24mm apart, USB-C at the pin 1 end",
          "ESP32-C3-Zero", (-1.38, -1.59, 16.62, 21.91), (7.62, 10.16), it)

# ---- single-row modules: row along +x, module body towards -y, pin names on the +y side
def single(name, descr, value, pins, body, label):
    n = len(pins); it = []
    for i, (num, nm, _) in enumerate(pins):
        nm = {"CAN_RX": "RX", "CAN_TX": "TX"}.get(nm, nm)
        it += [pad(num, i * 2.54, 0), txt(nm, i * 2.54, 2.7 + 0.37 * len(nm), 90, "F.SilkS")]
    cx = (body[0] + body[2]) / 2; cy = (body[1] + body[3]) / 2
    it += [txt(label, cx, cy - 2, 0, "F.SilkS", 1.0), txt("outline approx - test fit", cx, cy + 4, 0, "F.Fab", 0.8)]
    footprint(name, descr, value, body, (cx, cy), it)

single("Module_ADS1115_1x10", "ADS1115 breakout on a 1x10 socket. Outline approximate (28 x 18mm)", "ADS1115",
       U2_PINS, (-2.57, -16.0, 25.43, 2.0), "ADS1115")
single("Module_MAX31855_1x06", "MAX31855 breakout on a 1x6 socket. Outline approximate (20 x 20mm), screw terminal at the far edge",
       "MAX31855", U3_PINS, (-3.65, -18.0, 16.35, 2.0), "MAX31855")
single("Module_SN65HVD230_1x04", "Waveshare SN65HVD230 CAN module on a 1x4 socket. Outline approximate (16 x 29.5mm), screw terminal at the far edge",
       "SN65HVD230", U4_PINS, (-4.19, -27.5, 11.81, 2.0), "CAN")
open(OUT + "/fp-lib-table", "w").write(
    '(fp_lib_table\n\t(version 7)\n\t(lib (name "SensorNode")(type "KiCad")(uri "${KIPRJMOD}/SensorNode.pretty")(options "")(descr "Sensor node module sockets"))\n)\n')
print("footprints written")
