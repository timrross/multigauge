# Generates sensor-node-smd.kicad_sch from design.py.
# Every pin gets a short stub ending in a net label or power symbol, so the drawing is a set of
# labelled blocks rather than a web of wires. The netlist check in build.sh proves it matches design.py.
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import *

KS = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"


def f(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def balanced(t, i):
    """Index just past the s-expression that starts at t[i] == '('."""
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
                if d == 0: return j + 1
        j += 1


_libs = {}


def extract(lib, name):
    if lib not in _libs: _libs[lib] = open("%s/%s.kicad_sym" % (KS, lib)).read()
    t = _libs[lib]
    i = t.find('\n\t(symbol "%s"\n' % name)
    assert i >= 0, (lib, name)
    return t[i + 1:balanced(t, i + 1)]


def children(block):
    """Top-level child s-expressions of a symbol block, as (start, end) spans."""
    out = []; i = block.index("\n") + 1
    while True:
        j = block.find("(", i)
        if j < 0: break
        e = balanced(block, j); out.append((j, e)); i = e
    return out


def flatten(lib, name):
    """Library symbol text. A derived symbol becomes its parent's drawing with its own properties."""
    b = extract(lib, name)
    m = re.search(r'\(extends "([^"]+)"\)', b)
    if not m: return b
    parent = m.group(1); pb = flatten(lib, parent)
    own = [b[s:e] for s, e in children(b) if b[s:e].startswith("(property")]
    out = []; done = False
    last = pb.index("\n") + 1; head = pb[:last]
    for s, e in children(pb):
        chunk = pb[s:e]
        if chunk.startswith("(property"):
            if not done: out += own; done = True
            continue
        out.append(chunk)
    text = head + "".join("\t\t" + c + "\n" for c in out) + "\t)"
    return text.replace('(symbol "%s"' % parent, '(symbol "%s"' % name, 1).replace('(symbol "%s_' % parent, '(symbol "%s_' % name)


PIN_RE = re.compile(r'\(pin\s+(\S+)\s+(\S+)\s+\(at\s+(\S+)\s+(\S+)\s+(\S+)\)\s+\(length\s+(\S+)\).*?'
                    r'\(name\s+"([^"]*)".*?\(number\s+"([^"]*)"', re.S)

SYMS = {}
LIBS = sorted(set(v[1] for v in PARTS.values()) | {"power:+5V", "power:+3V3", "power:GND", "power:PWR_FLAG"})
for lid in LIBS:
    lib, name = lid.split(":")
    b = flatten(lib, name)
    pins = {p[7]: (float(p[2]), float(p[3]), int(float(p[4]))) for p in PIN_RE.findall(b)}
    SYMS[lid] = {"text": b.replace('(symbol "%s"' % name, '(symbol "%s:%s"' % (lib, name), 1), "pins": pins}


def tf(px, py, rot, mirror):
    if mirror == "y": px = -px
    if mirror == "x": py = -py
    return {0: (px, -py), 90: (-py, -px), 180: (-px, py), 270: (py, px)}[rot]


items = []; wires = []; conn_pts = []; placed = set(); used_syms = set()
cnt = {"pwr": 0, "flg": 0, "x": 0}


def inst(lib_id, ref, value, fp, x, y, rot, refp, valp, hide_ref=False, hide_val=False):
    sym = SYMS[lib_id]; used_syms.add(lib_id)
    u = uid("sym:" + ref)
    def prop(name, val, pos, hide):
        dx, dy, just = pos
        return '\t\t(property "%s" "%s" (at %s %s 0) %s(effects (font (size 1.27 1.27))%s))' % (
            name, val, f(x + dx), f(y + dy), "(hide yes) " if hide else "",
            (" (justify %s)" % just) if just else "")
    nobom = ref.startswith("#") or ref[:2] in ("TP", "JP") or ref[0] == "H"
    lines = ["\t(symbol", '\t\t(lib_id "%s")' % lib_id, "\t\t(at %s %s %d)" % (f(x), f(y), rot), "\t\t(unit 1)",
             "\t\t(exclude_from_sim no) (in_bom %s) (on_board %s) (dnp no)" % (
                 "no" if nobom else "yes", "no" if ref.startswith("#") else "yes"),
             '\t\t(uuid "%s")' % u,
             prop("Reference", ref, refp, hide_ref), prop("Value", value, valp, hide_val),
             prop("Footprint", fp, (0, 0, None), True), prop("Datasheet", "", (0, 0, None), True),
             prop("Description", "", (0, 0, None), True)]
    if ref in LCSC: lines.append(prop("LCSC", LCSC[ref], (0, 0, None), True))
    res = {}
    for num, (px, py, ang) in sym["pins"].items():
        dx, dy = tf(px, py, rot, None)
        ox, oy = {0: (-1, 0), 180: (1, 0), 90: (0, -1), 270: (0, 1)}[ang]   # outward, symbol coords (y up)
        ddx, ddy = tf(ox, oy, rot, None)
        res[num] = (round(x + dx, 4), round(y + dy, 4), ddx, ddy)
        conn_pts.append(res[num][:2])
        lines.append('\t\t(pin "%s" (uuid "%s"))' % (num, uid("pin:%s:%s" % (ref, num))))
    lines += ['\t\t(instances (project "%s" (path "/%s" (reference "%s") (unit 1))))' % (PROJECT, ROOT_UUID, ref), "\t)"]
    items.append("\n".join(lines))
    return res


def power(kind, x, y, rot=0):
    lib_id = "power:" + kind
    if kind == "PWR_FLAG":
        cnt["flg"] += 1; ref = "#FLG%02d" % cnt["flg"]
    else:
        cnt["pwr"] += 1; ref = "#PWR%02d" % cnt["pwr"]
    up = (0, -1) if kind == "GND" else (0, 1)
    dx, dy = tf(up[0], up[1], rot, None)
    if dx: valp = (dx * (3.81 if kind != "GND" else 4.2), 0, "left" if dx > 0 else "right")
    else: valp = (0, dy * (3.556 if kind != "GND" else 4.4), None)
    inst(lib_id, ref, kind, "", x, y, rot, (0, 0, None), valp, hide_ref=True)


def wire(a, b):
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


def nc(pt):
    cnt["x"] += 1
    items.append('\t(no_connect (at %s %s) (uuid "%s"))' % (f(pt[0]), f(pt[1]), uid("nc%d" % cnt["x"])))


def text(s, x, y, size=1.27, bold=False):
    cnt["x"] += 1
    items.append('\t(text "%s" (exclude_from_sim no) (at %s %s 0) (effects (font (size %s %s)%s) (justify left bottom)) (uuid "%s"))'
                 % (s.replace('"', "'"), f(x), f(y), f(size), f(size), " (bold yes)" if bold else "", uid("tx%d" % cnt["x"])))


def terminate(net, x, y, dx, dy):
    """Put a power symbol or a net label at the end of a stub that runs in direction (dx, dy)."""
    side = "L" if dx < 0 else "R" if dx > 0 else "U" if dy < 0 else "D"
    # Power symbols only where they read naturally (rail up, ground down); a label everywhere else.
    if net in ("+5V", "+3V3") and side == "U": power(net, x, y, 0)
    elif net == "GND" and side == "D": power("GND", x, y, 0)
    else: glabel(net, x, y, side)


def place(ref, x, y, rot=0, refp=None, valp=None, stub=5.08, hide_val=False):
    """Place a part and give every pin a stub with its net label (or a no-connect flag)."""
    value, lib_id, fp = PARTS[ref]; placed.add(ref)
    refp = refp or (2.54, -1.27, "left"); valp = valp or (2.54, 1.27, "left")
    pins = inst(lib_id, ref, value, fp, x, y, rot, refp, valp, hide_val=hide_val)
    seen = {}
    for num, (px, py, dx, dy) in pins.items():
        net = PIN_NET.get((ref, num))
        if (px, py) in seen:
            assert seen[(px, py)] == net, (ref, num)   # stacked pins must share a net
            continue
        seen[(px, py)] = net
        if net is None:
            assert (ref, num) in NO_CONNECT or lib_id == "Mechanical:MountingHole", (ref, num)
            nc((px, py)); continue
        end = (px + dx * stub, py + dy * stub)
        wire((px, py), end); terminate(net, end[0], end[1], dx, dy)
    return pins


def vpart(ref, x, y):
    """A two-pin part drawn vertically, turned so ground is at the bottom and a supply rail at the top."""
    a = PIN_NET.get((ref, "1")); b = PIN_NET.get((ref, "2"))
    flip = a == "GND" or b in ("+5V", "+3V3")
    place(ref, x, y, 180 if flip else 0, stub=2.54)


def hpart(ref, x, y, rot=0):
    place(ref, x, y, rot, refp=(0, -3.3, None), valp=(0, 3.6, None), stub=2.54)


def flag(net, x, y):
    power("PWR_FLAG", x, y)
    if net in ("+5V", "+3V3"): power(net, x, y, 180)
    elif net == "GND": power("GND", x, y)
    else: glabel(net, x, y, "D")


def heading(s, x, y, note=None):
    text(s, x, y, 1.8, True)
    if note: text(note, x, y + 3.6)


# ================================================================ ROW A
heading("LOOM CONNECTOR J1: Molex Micro-Fit 3.0, 2 x 8", 20.32, 25.4,
        "Pin n and pin n+8 share a column in the plug, so each sensor's wires sit together.")
place("J1", 53.34, 58.42, refp=(1.27, -12.7, None), valp=(1.27, 15.24, None))
for i, pin in enumerate(sorted(J1_MAP, key=int)):
    net, note = J1_MAP[pin]
    text("%2s  %s" % (pin, note), 91.44, 40.64 + i * 3.3)

heading("POWER: 5V in, filter, and 3.3V regulator", 149.86, 25.4,
        "D1 and D2 let the buck converter and USB be connected at the same time. Sensors run from +5V, logic from +3V3.")
vpart("L1", 157.48, 55.88); vpart("C1", 175.26, 55.88); vpart("C2", 190.5, 55.88)
hpart("D1", 215.9, 45.72); hpart("D2", 215.9, 66.04)
place("U5", 254, 55.88, refp=(-7.62, -8.89, "left"), valp=(-7.62, 8.89, "left"))
vpart("C7", 233.68, 81.28); vpart("C8", 276.86, 81.28)
flag("+5V", 157.48, 88.9); flag("GND", 172.72, 88.9); flag("VSYS", 187.96, 88.9)
text("Power flags (tell the rule checker these nets are supplied)", 149.86, 100.33)

heading("USB-C: flashing and serial monitor", 307.34, 25.4, "5k1 on CC1 and CC2 marks the board as a USB device.")
place("J3", 330.2, 63.5, refp=(-10.16, -19.05, "left"), valp=(-10.16, 21.59, "left"))
vpart("R13", 375.92, 55.88); vpart("R14", 393.7, 55.88)

# ================================================================ ROW B
heading("MCU: ESP32-C3-WROOM-02U module (same GPIO numbers as the rev 2 firmware)", 20.32, 111.76,
        "IO2 and IO8 must be high at power-up. BOOT held low at reset enters flashing mode.")
place("U1", 63.5, 157.48, refp=(-12.7, -24.13, "left"), valp=(-12.7, 24.13, "left"))
vpart("R9", 109.22, 137.16); vpart("C11", 124.46, 137.16); hpart("SW1", 147.32, 137.16)
vpart("R10", 109.22, 170.18); hpart("SW2", 132.08, 170.18)
vpart("R11", 160.02, 170.18); vpart("R12", 175.26, 170.18)
vpart("C9", 170.18, 137.16); vpart("C10", 185.42, 137.16)

heading("ADC: ADS1115, address 0x48", 203.2, 111.76, "ADDR to GND. 4k7 pull-ups on SDA and SCL.")
place("U2", 238.76, 149.86, refp=(-7.62, -13.97, "left"), valp=(-7.62, 13.97, "left"))
vpart("C12", 271.78, 147.32); vpart("R15", 271.78, 180.34); vpart("R16", 287.02, 180.34)

heading("ANALOGUE FRONT END: every ADC input stays below 3.3V", 307.34, 111.76,
        "Pressure: 10k over 15k divider. NTC: pull-up to 3V3, 10k series. 100nF filter on each.")
for i, ref in enumerate(("R1", "R2", "C3", "R3", "R4", "C4")): vpart(ref, 314.96 + i * 17.78, 142.24)
for i, ref in enumerate(("R5", "R7", "C5", "R6", "R8", "C6")): vpart(ref, 314.96 + i * 17.78, 185.42)

# ================================================================ ROW C
heading("EGT: MAX31855K thermocouple converter", 20.32, 213.36, "Probe goes straight to J2. Keep the probe insulated from ground.")
place("U3", 55.88, 246.38, refp=(-7.62, -11.43, "left"), valp=(-7.62, 11.43, "left"))
place("J2", 101.6, 246.38, rot=180, refp=(0, -6.35, None), valp=(0, 6.35, None))
vpart("C13", 121.92, 243.84); vpart("C14", 137.16, 243.84)

heading("CAN: SN65HVD230 transceiver, 500 kbit/s", 154.94, 213.36,
        "R17 terminates the bus. Cut JP1 to remove the termination.")
place("U4", 190.5, 246.38, refp=(-7.62, -8.89, "left"), valp=(-7.62, 11.43, "left"))
vpart("C15", 223.52, 243.84); vpart("R17", 238.76, 243.84); hpart("JP1", 269.24, 259.08)

heading("POWER LED, TEST POINTS, MOUNTING HOLES", 287.02, 213.36, "The LED runs from VSYS, so it lights on buck or USB power.")
vpart("R18", 292.1, 233.68); hpart("D3", 309.88, 233.68)
for i in range(9):
    ref = "TP%d" % (i + 1)
    place(ref, 327.66 + i * 10.16, 226.06, refp=(0, -5.08, None), valp=(0, 0, None), stub=2.54, hide_val=True)
for i in range(4):
    ref = "H%d" % (i + 1)
    place(ref, 330.2 + i * 12.7, 246.38, refp=(2.54, -1.27, "left"), valp=(2.54, 1.27, "left"))

assert placed == set(PARTS), set(PARTS) ^ placed

# ---------------------------------------------------------------- wires and junctions
deg = {}
for a, b in wires:
    deg[a] = deg.get(a, 0) + 1; deg[b] = deg.get(b, 0) + 1
for i, (a, b) in enumerate(wires):
    items.append('\t(wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid "%s"))'
                 % (f(a[0]), f(a[1]), f(b[0]), f(b[1]), uid("w%d" % i)))

head = ['(kicad_sch', '\t(version 20250114)', '\t(generator "eeschema")', '\t(generator_version "9.0")',
        '\t(uuid "%s")' % ROOT_UUID, '\t(paper "A3")',
        '\t(title_block (title "MultiGauge sensor node, SMD version") (date "2026-11-11") (rev "3")'
        ' (comment 1 "VW T4 digital dash, engine bay node") (comment 2 "Same circuit and firmware pins as the rev 2 carrier board"))',
        '\t(lib_symbols']
body = [SYMS[k]["text"] for k in sorted(used_syms)]
tail = ['\t)'] + items + ['\t(sheet_instances (path "/" (page "1")))', '\t(embedded_fonts no)', ')', '']
open("%s/%s.kicad_sch" % (OUT, PROJECT), "w").write("\n".join(head + body + tail))
print("wrote schematic:", len(items), "items")
