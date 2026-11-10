# Generates sensor-node-carrier.kicad_sch + SensorNode.kicad_sym from design.py
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import *

KS = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"


def f(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def extract(lib, name):
    t = open("%s/%s.kicad_sym" % (KS, lib)).read()
    i = t.find('\n\t(symbol "%s"\n' % name)
    assert i >= 0, name
    i += 1
    d = 0; j = i; s = False
    while True:
        c = t[j]
        if s:
            if c == "\\": j += 1
            elif c == '"': s = False
        else:
            if c == '"': s = True
            elif c == "(": d += 1
            elif c == ")":
                d -= 1
                if d == 0: break
        j += 1
    return t[i:j + 1]


PIN_RE = re.compile(r'\(pin\s+(\S+)\s+(\S+)\s+\(at\s+(\S+)\s+(\S+)\s+(\S+)\)\s+\(length\s+(\S+)\).*?'
                    r'\(name\s+"([^"]*)".*?\(number\s+"([^"]*)"', re.S)

SYMS = {}
for lib, name in [("Device", "R"), ("Device", "C"), ("Device", "C_Polarized"), ("Device", "L"),
                  ("Device", "D_Schottky"), ("Connector", "Screw_Terminal_01x12"),
                  ("Connector", "Screw_Terminal_01x02"), ("Connector_Generic", "Conn_01x02"),
                  ("Connector_Generic", "Conn_01x04"), ("Connector", "TestPoint"),
                  ("Mechanical", "MountingHole"), ("power", "+5V"), ("power", "+3V3"), ("power", "GND"),
                  ("power", "PWR_FLAG")]:
    b = extract(lib, name)
    assert "(extends" not in b
    pins = {p[7]: (float(p[2]), float(p[3])) for p in PIN_RE.findall(b)}
    SYMS["%s:%s" % (lib, name)] = {
        "text": b.replace('(symbol "%s"' % name, '(symbol "%s:%s"' % (lib, name), 1), "pins": pins}

E = "(effects (font (size 1.27 1.27)))"
EH = "(hide yes) (effects (font (size 1.27 1.27)))"


def modsym(name, left, right, w, value, fp, desc):
    n = max(len(left), len(right)); top = (n - 1) * 1.27; hh = top + 2.54
    pins = {}; ptxt = []
    for side, lst in (("L", left), ("R", right)):
        for i, (num, pn, et) in enumerate(lst):
            y = top - i * 2.54
            x = -(w / 2 + 5.08) if side == "L" else (w / 2 + 5.08)
            ptxt.append('\t\t\t(pin %s line (at %s %s %d) (length 5.08) (name "%s" %s) (number "%s" %s))'
                        % (et, f(x), f(y), 0 if side == "L" else 180, pn, E, num, E))
            pins[num] = (x, y)
    def text(full):
        return "\n".join([
            '\t\t(symbol "%s"' % full,
            "\t\t\t(pin_names (offset 1.016))",
            "\t\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)",
            '\t\t\t(property "Reference" "U" (at %s %s 0) (effects (font (size 1.27 1.27)) (justify left)))' % (f(-w / 2), f(hh + 1.27)),
            '\t\t\t(property "Value" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) (justify left)))' % (value, f(-w / 2), f(-hh - 1.27)),
            '\t\t\t(property "Footprint" "%s" (at 0 0 0) %s)' % (fp, EH),
            '\t\t\t(property "Datasheet" "" (at 0 0 0) %s)' % EH,
            '\t\t\t(property "Description" "%s" (at 0 0 0) %s)' % (desc, EH),
            '\t\t\t(symbol "%s_0_1"' % name,
            "\t\t\t\t(rectangle (start %s %s) (end %s %s) (stroke (width 0.254) (type default)) (fill (type background)))"
            % (f(-w / 2), f(hh), f(w / 2), f(-hh)),
            "\t\t\t)",
            '\t\t\t(symbol "%s_1_1"' % name,
        ] + ptxt + ["\t\t\t)", "\t\t)"])
    return {"text": text("SensorNode:" + name), "libtext": text(name), "pins": pins, "w": w, "hh": hh}


MODS = [
    ("ESP32-C3-Zero", U1_LEFT, U1_RIGHT, 20.32, "U1", "Waveshare ESP32-C3-Zero on two 1x9 rows, 15.24mm apart"),
    ("ADS1115_Module", U2_PINS, [], 15.24, "U2", "ADS1115 16-bit ADC breakout, 1x10 header"),
    ("MAX31855_Module", U3_PINS, [], 15.24, "U3", "MAX31855 K-type thermocouple breakout, 1x6 header"),
    ("CAN_Module", U4_PINS, [], 15.24, "U4", "Waveshare SN65HVD230 CAN transceiver module, 1x4 header"),
]
for name, l, r, w, ref, desc in MODS:
    SYMS["SensorNode:" + name] = modsym(name, l, r, w, PARTS[ref][0], PARTS[ref][2], desc)


def tf(px, py, rot, mirror):
    if mirror == "y": px = -px
    if mirror == "x": py = -py
    return {0: (px, -py), 90: (-py, -px), 180: (-px, py), 270: (py, px)}[rot]


items = []; wires = []; conn_pts = []; placed = set(); used_syms = set()
cnt = {"pwr": 0, "flg": 0, "w": 0, "x": 0}


def inst(lib_id, ref, value, fp, x, y, rot, mirror, refp, valp, hide_ref=False, hide_val=False, desc=""):
    sym = SYMS[lib_id]; used_syms.add(lib_id)
    fa = 90 if rot in (90, 270) else 0
    u = uid("sym:" + ref)
    def prop(name, val, pos, hide):
        dx, dy, just = pos
        return '\t\t(property "%s" "%s" (at %s %s %d) %s(effects (font (size 1.27 1.27))%s))' % (
            name, val, f(x + dx), f(y + dy), fa, "(hide yes) " if hide else "",
            (" (justify %s)" % just) if just else "")
    lines = ["\t(symbol", '\t\t(lib_id "%s")' % lib_id, "\t\t(at %s %s %d)" % (f(x), f(y), rot)]
    if mirror: lines.append("\t\t(mirror %s)" % mirror)
    lines += ["\t\t(unit 1)", "\t\t(exclude_from_sim no) (in_bom %s) (on_board %s) (dnp no)" % (
        ("no", "no") if ref.startswith("#") else (("no", "yes") if ref[0] in "TH" else ("yes", "yes"))),
        '\t\t(uuid "%s")' % u,
        prop("Reference", ref, refp, hide_ref), prop("Value", value, valp, hide_val),
        prop("Footprint", fp, (0, 0, None), True), prop("Datasheet", "", (0, 0, None), True),
        prop("Description", desc, (0, 0, None), True)]
    res = {}
    for num, (px, py) in sym["pins"].items():
        dx, dy = tf(px, py, rot, mirror)
        res[num] = (round(x + dx, 4), round(y + dy, 4))
        conn_pts.append(res[num])
        lines.append('\t\t(pin "%s" (uuid "%s"))' % (num, uid("pin:%s:%s" % (ref, num))))
    lines += ['\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))' % (PROJECT, ROOT_UUID, ref), "\t)"]
    items.append("\n".join(lines))
    return res


def place(ref, x, y, rot=0, mirror=None, refp=None, valp=None, hide_val=False):
    value, lib_id, fp = PARTS[ref]; placed.add(ref)
    sym = SYMS[lib_id]
    if "w" in sym:
        refp = refp or (-sym["w"] / 2, -sym["hh"] - 1.27, "left")
        valp = valp or (-sym["w"] / 2, sym["hh"] + 1.27, "left")
    elif rot in (90, 180, 270) and lib_id.startswith("Device"):
        refp = refp or (0, -2.54, None); valp = valp or (0, 2.54, None)
    else:
        refp = refp or (2.54, -1.27, "left"); valp = valp or (2.54, 1.27, "left")
    return inst(lib_id, ref, value, fp, x, y, rot, mirror, refp, valp, hide_val=hide_val)


def power(kind, x, y, rot=0):
    lib_id = "power:" + kind
    if kind == "PWR_FLAG":
        cnt["flg"] += 1; ref = "#FLG%02d" % cnt["flg"]
    else:
        cnt["pwr"] += 1; ref = "#PWR%02d" % cnt["pwr"]
    up = (0, -1) if kind == "GND" else (0, 1)
    dx, dy = tf(up[0], up[1], rot, None)
    if dx:
        just = "left" if dx > 0 else "right"
        if rot == 90: just = "right" if just == "left" else "left"
        valp = (dx * (3.81 if kind != "GND" else 4.2), 0, just)
    else:
        valp = (0, dy * (3.556 if kind != "GND" else 4.4), None)
    r = inst(lib_id, ref, kind, "", x, y, rot, None, (0, 0, None), valp, hide_ref=True,
             hide_val=(kind == "PWR_FLAG" and False))
    # power symbol value fields must stay horizontal regardless of symbol rotation
    return r["1"]


def wire(*pts):
    for a, b in zip(pts, pts[1:]):
        a = (round(a[0], 4), round(a[1], 4)); b = (round(b[0], 4), round(b[1], 4))
        assert a[0] == b[0] or a[1] == b[1], (a, b)
        if a != b: wires.append((a, b))


def glabel(name, x, y, side):
    ang, just = {"L": (180, "right"), "R": (0, "left"), "U": (90, "left"), "D": (270, "right")}[side]
    cnt["x"] += 1
    conn_pts.append((round(x, 4), round(y, 4)))
    items.append('\t(global_label "%s" (shape passive) (at %s %s %d) (effects (font (size 1.27 1.27)) (justify %s)) (uuid "%s")\n'
                 '\t\t(property "Intersheetrefs" "${INTERSHEET_REFS}" (at 0 0 0) (hide yes) (effects (font (size 1.27 1.27))))\n\t)'
                 % (name, f(x), f(y), ang, just, uid("gl%d" % cnt["x"])))


def label(name, x, y):
    cnt["x"] += 1
    conn_pts.append((round(x, 4), round(y, 4)))
    items.append('\t(label "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) (justify left bottom)) (uuid "%s"))'
                 % (name, f(x), f(y), uid("lb%d" % cnt["x"])))


def nc(pt):
    cnt["x"] += 1
    items.append('\t(no_connect (at %s %s) (uuid "%s"))' % (f(pt[0]), f(pt[1]), uid("nc%d" % cnt["x"])))


def text(s, x, y, size=1.27, bold=False):
    cnt["x"] += 1
    items.append('\t(text "%s" (exclude_from_sim no) (at %s %s 0) (effects (font (size %s %s)%s) (justify left bottom)) (uuid "%s"))'
                 % (s.replace('"', "'"), f(x), f(y), f(size), f(size), " (bold yes)" if bold else "", uid("tx%d" % cnt["x"])))


def stub(pin, dx, net):
    """Wire a horizontal stub from a pin and terminate it with a power symbol or a global label."""
    end = (pin[0] + dx, pin[1]); wire(pin, end)
    left = dx < 0
    if net in ("+5V", "+3V3"): power(net, end[0], end[1], 90 if left else 270)
    elif net == "GND": power("GND", end[0], end[1], 270 if left else 90)
    else: glabel(net, end[0], end[1], "L" if left else "R")


# ---------------------------------------------------------------- POWER
text("POWER IN: 5V from the buck converter, filtered by L1 / C1 / C2", 20.32, 25.4, 1.8, True)
text("D1 lets USB and the buck be connected together.", 20.32, 28.6)
j2 = place("J2", 25.4, 38.1, mirror="y", refp=(-2.54, -3.81, None), valp=(-2.54, 7.62, None))
l1 = place("L1", 44.45, 38.1, rot=90)
c1 = place("C1", 58.42, 44.45); c2 = place("C2", 72.39, 44.45)
d1 = place("D1", 86.36, 38.1, rot=180)
wire(j2["1"], l1["1"]); label("5V_IN", 31.75, 38.1)
wire(l1["2"], d1["2"])
wire((58.42, 38.1), c1["1"]); wire((72.39, 38.1), c2["1"])
power("PWR_FLAG", 53.34, 38.1); power("+5V", 78.74, 38.1)
wire(d1["1"], (99.06, 38.1)); power("PWR_FLAG", 93.98, 38.1); glabel("+5V_MCU", 99.06, 38.1, "R")
wire(j2["2"], (30.48, 50.8), (72.39, 50.8)); wire(c1["2"], (58.42, 50.8)); wire(c2["2"], (72.39, 50.8))
wire((45.72, 50.8), (45.72, 53.34)); power("GND", 45.72, 53.34)
power("PWR_FLAG", 38.1, 50.8)

# ---------------------------------------------------------------- ANALOGUE FRONT END
text("ANALOGUE FRONT END: every ADC input stays below 3.3V", 20.32, 66.04, 1.8, True)
X0 = 38.1


def pressure(y, raw, out, ru, rl, c, note):
    text(note, X0 - 15.24, y - 6.35)
    a = place(ru, X0 + 7.62, y, rot=90); b = place(rl, X0 + 15.24, y + 6.35); cc = place(c, X0 + 27.94, y + 6.35)
    glabel(raw, X0, y, "L"); wire((X0, y), a["1"]); wire(a["2"], (X0 + 35.56, y)); glabel(out, X0 + 35.56, y, "R")
    wire((X0 + 15.24, y), b["1"]); wire((X0 + 27.94, y), cc["1"])
    power("GND", b["2"][0], b["2"][1]); power("GND", cc["2"][0], cc["2"][1])


def ntc(y, raw, out, rp, rs, c, note):
    text(note, X0 - 15.24, y - 6.35)
    p = place(rp, X0 + 7.62, y - 6.35, refp=(2.54, -1.27, "left"), valp=(2.54, 1.27, "left"))
    s = place(rs, X0 + 19.05, y, rot=90)
    cc = place(c, X0 + 27.94, y + 6.35)
    glabel(raw, X0, y, "L"); wire((X0, y), s["1"]); wire(p["2"], (X0 + 7.62, y))
    power("+3V3", p["1"][0], p["1"][1])
    wire(s["2"], (X0 + 35.56, y)); glabel(out, X0 + 35.56, y, "R")
    wire((X0 + 27.94, y), cc["1"]); power("GND", cc["2"][0], cc["2"][1])


pressure(78.74, "OIL_P_RAW", "OIL_P", "R1", "R2", "C3", "Oil pressure 0.5-4.5V x0.6")
pressure(101.6, "BOOST_RAW", "BOOST", "R3", "R4", "C4", "Boost 0-5V x0.6")
ntc(133.35, "OIL_T_RAW", "OIL_T", "R5", "R7", "C5", "Oil temp NTC")
ntc(162.56, "IAT_RAW", "IAT", "R6", "R8", "C6", "Intake temp NTC")

# ---------------------------------------------------------------- MCU
text("MCU: Waveshare ESP32-C3-Zero", 119.38, 25.4, 1.8, True)
text("Pins 1-9 = left row, 10-18 = right row, both counted from the USB end.", 119.38, 28.6)
u1 = place("U1", 154.94, 55.88)
for p in U1_LEFT + U1_RIGHT:
    num = p[0]; net = PIN_NET.get(("U1", num))
    if net is None: nc(u1[num])
    else: stub(u1[num], -5.08 if int(num) <= 9 else 5.08, net)

# ---------------------------------------------------------------- ADC
text("ADC: ADS1115 module, address 0x48 (ADDR to GND)", 119.38, 88.9, 1.8, True)
u2 = place("U2", 154.94, 109.22)
for num, _, _ in U2_PINS:
    net = PIN_NET.get(("U2", num))
    if net is None: nc(u2[num])
    else: stub(u2[num], -5.08, net)

text("I2C expansion (BME280 or second ADS1115)", 119.38, 132.08, 1.8, True)
j4 = place("J4", 154.94, 140.97, refp=(0, -6.35, None), valp=(0, 8.89, None))
for num in "1234": stub(j4[num], -5.08, PIN_NET[("J4", num)])

# ---------------------------------------------------------------- right column
text("EGT: MAX31855 module (probe on its own terminal)", 198.12, 25.4, 1.8, True)
u3 = place("U3", 236.22, 40.64)
for num, _, _ in U3_PINS:
    net = PIN_NET.get(("U3", num))
    if net is None: nc(u3[num])
    else: stub(u3[num], -5.08, net)

text("CAN: SN65HVD230 module (bus on its own terminal)", 198.12, 59.69, 1.8, True)
u4 = place("U4", 236.22, 71.12)
for num, _, _ in U4_PINS: stub(u4[num], -5.08, PIN_NET[("U4", num)])
text("If the bus stays silent, swap GP0 and GP1 in firmware.", 198.12, 82.55)

text("LOOM: 12-way terminal to the Deutsch DT connector", 198.12, 93.98, 1.8, True)
j1 = place("J1", 236.22, 115.57, refp=(0, -16.51, None), valp=(0, 19.05, None))
J1_NOTES = ["+5V  oil pressure sensor", "GND  oil pressure sensor", "Oil pressure signal", "Boost signal",
            "Oil temp NTC", "Oil temp NTC return", "Intake temp NTC", "Intake temp NTC return",
            "+5V  boost sensor", "GND  boost sensor", "Spare", "Spare"]
for i in range(12):
    num = str(i + 1); stub(j1[num], -5.08, PIN_NET[("J1", num)])
    text(J1_NOTES[i], 242.57, j1[num][1] + 0.635)
j3 = place("J3", 236.22, 149.86, refp=(0, -3.81, None), valp=(0, 6.35, None))
for num in "12": stub(j3[num], -5.08, PIN_NET[("J3", num)])

# ---------------------------------------------------------------- test points + holes
text("TEST POINTS", 119.38, 160.02, 1.8, True)
for i in range(7):
    ref = "TP%d" % (i + 1); x = 121.92 + i * 7.62; y = 168.91
    tp = place(ref, x, y, refp=(0, -5.08, None), valp=(0, 0, None), hide_val=True)
    net = PIN_NET[(ref, "1")]
    wire(tp["1"], (x, y + 2.54))
    if net == "GND": power("GND", x, y + 2.54)
    elif net in ("+5V", "+3V3"): power(net, x, y + 2.54, 180)
    else: glabel(net, x, y + 2.54, "D")
text("MOUNTING HOLES (M3)", 119.38, 187.96, 1.8, True)
for i in range(4):
    ref = "H%d" % (i + 1)
    inst("Mechanical:MountingHole", ref, PARTS[ref][0], PARTS[ref][2], 121.92 + i * 12.7, 193.04, 0, None,
         (2.54, -1.27, "left"), (2.54, 1.27, "left")); placed.add(ref)

assert placed == set(PARTS), set(PARTS) ^ placed

# ---------------------------------------------------------------- split wires at connection points, add junctions
pts = set(conn_pts)
for a, b in wires: pts.add(a); pts.add(b)


def on_interior(p, a, b):
    if a[0] == b[0] == p[0]: return min(a[1], b[1]) < p[1] < max(a[1], b[1])
    if a[1] == b[1] == p[1]: return min(a[0], b[0]) < p[0] < max(a[0], b[0])
    return False


final = []
for a, b in wires:
    cuts = sorted([p for p in pts if on_interior(p, a, b)], key=lambda p: (abs(p[0] - a[0]) + abs(p[1] - a[1])))
    chain = [a] + cuts + [b]
    final += list(zip(chain, chain[1:]))
deg = {}
for a, b in final:
    deg[a] = deg.get(a, 0) + 1; deg[b] = deg.get(b, 0) + 1
for p in conn_pts: deg[p] = deg.get(p, 0) + 1
for i, (a, b) in enumerate(final):
    items.append('\t(wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid "%s"))'
                 % (f(a[0]), f(a[1]), f(b[0]), f(b[1]), uid("w%d" % i)))
for i, (p, n) in enumerate(sorted(deg.items())):
    if n >= 3:
        items.append('\t(junction (at %s %s) (diameter 0) (color 0 0 0 0) (uuid "%s"))' % (f(p[0]), f(p[1]), uid("j%d" % i)))

# ---------------------------------------------------------------- write files
os.makedirs(OUT, exist_ok=True)
head = ['(kicad_sch', '\t(version 20250114)', '\t(generator "eeschema")', '\t(generator_version "9.0")',
        '\t(uuid "%s")' % ROOT_UUID, '\t(paper "A4")',
        '\t(title_block (title "MultiGauge sensor node carrier") (date "2026-11-10") (rev "2")'
        ' (comment 1 "VW T4 digital dash, engine bay node") (comment 2 "See docs/SENSOR_NODE_HANDOFF.md"))',
        '\t(lib_symbols']
body = [SYMS[k]["text"] for k in sorted(used_syms)]
tail = ['\t)'] + items + ['\t(sheet_instances (path "/" (page "1")))', '\t(embedded_fonts no)', ')', '']
open("%s/%s.kicad_sch" % (OUT, PROJECT), "w").write("\n".join(head + body + tail))

lib = ['(kicad_symbol_lib', '\t(version 20241209)', '\t(generator "kicad_symbol_editor")', '\t(generator_version "9.0")']
lib += [SYMS["SensorNode:" + m[0]]["libtext"].replace("\n\t\t", "\n\t").replace("\t\t(symbol", "\t(symbol", 1) for m in MODS]
open("%s/SensorNode.kicad_sym" % OUT, "w").write("\n".join(lib + [")", ""]))
open("%s/sym-lib-table" % OUT, "w").write(
    '(sym_lib_table\n\t(version 7)\n\t(lib (name "SensorNode")(type "KiCad")(uri "${KIPRJMOD}/SensorNode.kicad_sym")(options "")(descr "Sensor node module symbols"))\n)\n')
print("wrote schematic:", len(items), "items")
