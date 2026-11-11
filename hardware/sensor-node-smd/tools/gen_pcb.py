# Generates sensor-node-smd.kicad_pcb: placement, outline, silk, routing, GND pours.
# Run with KiCad's bundled python (needs pcbnew).
#   gen_pcb.py <netlist> [--no-route]
import os, sys, math, heapq, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbnew
from design import *
from sexp import parse, find, get

ROUTE = "--no-route" not in sys.argv
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
NETFILE = sys.argv[1]
W, H = 46.0, 48.0            # board size, mm
OX, OY = 100.0, 60.0         # board origin on the page
mm = pcbnew.FromMM; tomm = pcbnew.ToMM


def V(x, y): return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


# ref: (footprint origin x, y, rotation). Keep every part on a 0.1mm position.
PLACE = {
    # connectors and holes
    "J1": (33.5, 39.0, 180), "J2": (40.8, 26.8, 90), "J3": (3.7, 12.85, 270),
    "H1": (3.7, 3.7, 0), "H2": (42.3, 3.7, 0), "H3": (3.7, 44.3, 0), "H4": (42.3, 44.3, 0),
    # MCU and its support parts
    "U1": (22.0, 7.8, 180),
    "SW2": (35.3, 2.9, 0), "R10": (33.5, 5.4, 0), "R11": (36.7, 5.4, 0),
    "C11": (33.5, 8.8, 0), "R9": (33.5, 10.6, 180), "SW1": (42.6, 9.6, 0),
    "C10": (33.5, 13.8, 0), "C9": (37.2, 13.9, 0), "R12": (16.5, 17.2, 0),
    "D3": (16.2, 22.0, 0), "R18": (19.6, 22.6, 180),
    # USB
    "R13": (10.3, 10.0, 90), "R14": (10.3, 16.3, 270),
    # power
    "L1": (4.0, 36.5, 180), "C1": (4.2, 28.2, 270), "C2": (8.9, 30.0, 90),
    "D2": (2.8, 19.8, 180), "D1": (2.8, 22.2, 180), "C7": (6.3, 20.6, 90),
    "U5": (9.5, 20.4, 0), "C8": (12.8, 20.2, 270),
    # CAN
    "U4": (15.5, 29.5, 270), "C15": (15.5, 24.8, 0), "R17": (11.8, 31.5, 90), "JP1": (11.4, 27.6, 90),
    # thermocouple
    "U3": (29.7, 20.0, 180), "C13": (34.3, 16.2, 0), "C14": (34.25, 20.0, 90),
    # ADC and analogue front end
    "U2": (27.4, 25.2, 0), "C12": (31.6, 25.2, 270), "R15": (22.8, 18.6, 0), "R16": (22.8, 20.4, 0),
    "R1": (20.3, 32.6, 90), "R2": (22.1, 32.6, 270), "C3": (20.3, 29.4, 90),
    "R3": (24.5, 32.6, 90), "R4": (26.3, 32.6, 270), "C4": (24.5, 29.4, 90),
    "R7": (28.6, 32.6, 90), "R5": (30.4, 32.6, 270), "C5": (28.6, 29.4, 90),
    "R8": (32.4, 32.6, 90), "R6": (34.2, 32.6, 270), "C6": (32.4, 29.4, 90),
    # test points
    "TP1": (9.2, 33.2, 0), "TP2": (37.6, 16.8, 0), "TP3": (40.2, 16.8, 0),
    "TP4": (22.2, 29.4, 0), "TP5": (26.4, 29.4, 0), "TP6": (30.5, 29.4, 0), "TP7": (33.6, 26.6, 0),
    "TP8": (19.8, 18.6, 0), "TP9": (19.8, 20.6, 0),
}
TP_LABEL = {"TP1": "5V", "TP2": "3V3", "TP3": "GND", "TP4": "A0", "TP5": "A1", "TP6": "A2", "TP7": "A3",
            "TP8": "SDA", "TP9": "SCL"}
FINE = {"U2": None, "J3": ("A5", "A6", "A7", "A8", "B5", "B6", "B7", "B8")}   # 0.3mm pads that need a straight escape

# ---- pad -> net from the schematic netlist (includes KiCad's names for unconnected pins)
nl = parse(open(NETFILE).read())
padnet = {}
for n in find(find(nl, "nets")[0], "net"):
    for x in find(n, "node"):
        padnet[(get(x, "ref"), get(x, "pin"))] = get(n, "name")
for k, v in PIN_NET.items(): assert padnet[k].lstrip("/") == v, (k, padnet[k])

board = pcbnew.CreateEmptyBoard()
nets = {}
for name in sorted(set(padnet.values())):
    ni = pcbnew.NETINFO_ITEM(board, name); board.Add(ni); nets[name] = ni


def text(s, x, y, size=0.8, rot=0, just=None, layer=pcbnew.F_SilkS):
    t = pcbnew.PCB_TEXT(board); t.SetText(s); t.SetPosition(V(x, y)); t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(max(0.1, size * 0.15)))
    if rot: t.SetTextAngleDegrees(rot)
    if just == "left": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    if just == "right": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    if layer == pcbnew.B_SilkS: t.SetMirrored(True)
    board.Add(t)


# ---- outline
e = pcbnew.PCB_SHAPE(board); e.SetShape(pcbnew.SHAPE_T_RECT); e.SetStart(V(0, 0)); e.SetEnd(V(W, H))
e.SetLayer(pcbnew.Edge_Cuts); e.SetWidth(mm(0.1)); board.Add(e)

# ---- footprints
PADS = []   # dicts: ref, num, net, x, y (board mm), hx, hy (half sizes), th (True = both layers), npth
FPS = {}
for ref, (value, sym, fpid) in PARTS.items():
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad((OUT + "/SensorNodeSMD.pretty") if lib == "SensorNodeSMD" else "%s/%s.pretty" % (FPDIR, lib), name)
    assert fp is not None, fpid
    px, py, rot = PLACE[ref]
    fp.SetReference(ref); fp.SetValue(value); fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetPosition(V(px, py)); fp.SetOrientationDegrees(rot)
    fp.SetPath(pcbnew.KIID_PATH("/" + uid("sym:" + ref)))
    fp.SetSheetname("/"); fp.SetSheetfile(PROJECT + ".kicad_sch")
    if ref in LCSC:
        fp.SetField("LCSC", LCSC[ref])
        for fld in fp.GetFields():
            if fld.GetName() == "LCSC": fld.SetVisible(False)
    board.Add(fp); FPS[ref] = fp
    fp.Reference().SetVisible(False)          # too dense for reference text; the fab layer keeps it
    for p in fp.Pads():
        num = p.GetNumber(); pos = p.GetPosition()
        x = tomm(pos.x) - OX; y = tomm(pos.y) - OY
        bb = p.GetBoundingBox(); hx = tomm(bb.GetWidth()) / 2; hy = tomm(bb.GetHeight()) / 2
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            keep = 3.2 if ref[0] == "H" else hx + 0.25     # room for an M3 screw head
            PADS.append(dict(ref=ref, num=num, net="__hole__", x=x, y=y, hx=keep, hy=keep, th=True, round=True)); continue
        if num == "" and not (p.IsOnLayer(pcbnew.F_Cu) or p.IsOnLayer(pcbnew.B_Cu)): continue   # paste-only aperture
        net = padnet[(ref, num)]
        p.SetNet(nets[net])
        if net.lstrip("/") == "GND" and ref[0] in "UJ" and not (ref in ("J1", "J2")):
            p.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)   # solid ground on chip and USB pads
        PADS.append(dict(ref=ref, num=num, net=net.lstrip("/"), x=x, y=y, hx=hx, hy=hy,
                         th=p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH, round=False))

# ---- board silk
TP_AT = {"TP8": (-1.9, 0), "TP9": (-1.9, 0)}      # label offset from the pad; default is just above it
for ref, s in TP_LABEL.items():
    px, py, _ = PLACE[ref]; dx, dy = TP_AT.get(ref, (0, -1.35))
    text(s, px + dx, py + dy, 0.8)
text("BOOT", 35.3, 0.75, 0.8); text("RST", 38.6, 9.6, 0.8)
text("T+", 37.2, 21.8, 0.8); text("T-", 37.2, 26.8, 0.8)
text("MultiGauge", 41.6, 33.0, 0.8); text("sensor node", 41.6, 34.4, 0.8); text("rev 3 SMD", 41.6, 35.8, 0.8)
text("2026-11", 41.6, 37.2, 0.8)
# J1 pin table on the underside
SHORT = {"5V_IN": "5V IN", "GND": "GND", "CANH": "CAN H", "CANL": "CAN L", "+5V": "5V out", "OIL_P_RAW": "OIL P",
         "BOOST_RAW": "BOOST", "OIL_T_RAW": "OIL T", "IAT_RAW": "IAT", None: "spare"}
text("J1 PINS", 23.0, 3.0, 1.0, layer=pcbnew.B_SilkS)
for k in range(8):
    a = str(k + 1); b = str(k + 9); y = 5.0 + k * 1.5
    text("%s  %s" % (a, SHORT[J1_MAP[a][0]]), 29.5, y, 0.8, layer=pcbnew.B_SilkS)
    text("%s  %s" % (b, SHORT[J1_MAP[b][0]]), 16.5, y, 0.8, layer=pcbnew.B_SilkS)
text("Square pad = pin 1", 23.0, 17.5, 0.8, layer=pcbnew.B_SilkS)
text("Pin 9 is behind pin 1", 23.0, 19.0, 0.8, layer=pcbnew.B_SilkS)

# ============================================================ router
G = 0.1
NX = int(round(W / G)) + 1; NY = int(round(H / G)) + 1
CL, M = 0.16, 0.06           # clearance, plus a margin so a segment between two nodes stays legal
VIA_R, VIA_DRILL = 0.3, 0.3
REACH = 0.75                 # obstacles further than this from a node are not recorded
SIG, PWR = 0.1, 0.25         # half widths: 0.2mm signal, 0.5mm power
VIA_COST = 30.0
LAYER_COST = [1.0, 1.6]      # F.Cu, B.Cu: stay on top so the bottom stays a ground plane
EDGE = 0.5
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
STEP = [1.0, 1.4142] * 4
INF = 1e9


def idx(i, j): return i * NY + j


class Field:
    """Per layer: nearest obstacle (distance, net) and nearest obstacle of any other net, at every grid node."""
    def __init__(self):
        n = NX * NY
        self.d1 = [[INF] * n, [INF] * n]; self.n1 = [[None] * n, [None] * n]; self.d2 = [[INF] * n, [INF] * n]

    def copy(self):
        c = Field.__new__(Field)
        c.d1 = [a[:] for a in self.d1]; c.n1 = [a[:] for a in self.n1]; c.d2 = [a[:] for a in self.d2]
        c.nv = bytearray(self.nv)
        return c

    def no_via_near(self, x, y, r):
        for i in range(max(0, int((x - r) / G)), min(NX - 1, int((x + r) / G) + 1) + 1):
            for j in range(max(0, int((y - r) / G)), min(NY - 1, int((y + r) / G) + 1) + 1):
                if math.hypot(i * G - x, j * G - y) < r: self.nv[idx(i, j)] = 1

    def put(self, l, k, d, net):
        d1 = self.d1[l]; n1 = self.n1[l]
        if n1[k] == net:
            if d < d1[k]: d1[k] = d
        elif d < d1[k]:
            self.d2[l][k] = d1[k]; d1[k] = d; n1[k] = net
        elif d < self.d2[l][k]:
            self.d2[l][k] = d

    def rect(self, layers, x, y, hx, hy, net, rnd=False):
        for i in range(max(0, int((x - hx - REACH) / G)), min(NX - 1, int((x + hx + REACH) / G) + 1) + 1):
            for j in range(max(0, int((y - hy - REACH) / G)), min(NY - 1, int((y + hy + REACH) / G) + 1) + 1):
                if rnd: d = math.hypot(i * G - x, j * G - y) - hx
                else:
                    dx = abs(i * G - x) - hx; dy = abs(j * G - y) - hy
                    d = math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 or dy > 0) else max(dx, dy)
                if d < REACH:
                    for l in layers: self.put(l, idx(i, j), d, net)

    def seg(self, l, ax, ay, bx, by, hw, net):
        vx, vy = bx - ax, by - ay; L2 = vx * vx + vy * vy
        for i in range(max(0, int((min(ax, bx) - hw - REACH) / G)), min(NX - 1, int((max(ax, bx) + hw + REACH) / G) + 1) + 1):
            for j in range(max(0, int((min(ay, by) - hw - REACH) / G)), min(NY - 1, int((max(ay, by) + hw + REACH) / G) + 1) + 1):
                x = i * G; y = j * G
                t = 0 if L2 == 0 else max(0, min(1, ((x - ax) * vx + (y - ay) * vy) / L2))
                d = math.hypot(x - ax - t * vx, y - ay - t * vy) - hw
                if d < REACH: self.put(l, idx(i, j), d, net)

    def blocked(self, l, k, net, need):
        return (self.d1[l][k] < need and self.n1[l][k] != net) or self.d2[l][k] < need


base = Field()
for p in PADS:
    base.rect((0, 1) if p["th"] else (0,), p["x"], p["y"], p["hx"], p["hy"], p["net"], p["round"])

# Where vias may not go: inside or close to any SMD pad (solder would wick away), except big ground pads.
novia = bytearray(NX * NY)
for p in PADS:
    if p["th"] or (p["net"] == "GND" and min(p["hx"], p["hy"]) >= 1.0): continue
    r = VIA_R + 0.15
    for i in range(max(0, int((p["x"] - p["hx"] - r) / G)), min(NX - 1, int((p["x"] + p["hx"] + r) / G) + 1) + 1):
        for j in range(max(0, int((p["y"] - p["hy"] - r) / G)), min(NY - 1, int((p["y"] + p["hy"] + r) / G) + 1) + 1):
            dx = abs(i * G - p["x"]) - p["hx"]; dy = abs(j * G - p["y"]) - p["hy"]
            if math.hypot(max(dx, 0), max(dy, 0)) < r: novia[idx(i, j)] = 1
# ... and not under the USB socket's metal shell
jx, jy, _ = PLACE["J3"]
for i in range(int((jx - 3.9) / G), int((jx + 3.9) / G) + 1):
    for j in range(int((jy - 4.7) / G), int((jy + 4.7) / G) + 1):
        if 0 <= i < NX and 0 <= j < NY: novia[idx(i, j)] = 1
base.nv = novia
# ... and not touching any drilled pad or hole, whatever its net (keeps holes apart)
for p in PADS:
    if p["th"]: base.no_via_near(p["x"], p["y"], max(p["hx"], p["hy"]) + VIA_R + 0.1)

# ---- terminals: one per pad group. Pads that share (ref, number) and overlap count as one.
fixed = []     # pre-made segments: (net, layer, ax, ay, bx, by, hw)
TERMS = {}     # net -> list of terminals: dict(node=(i, j), th=bool, ref, num)
groups = {}
for p in PADS:
    if p["net"] == "__hole__" or p["net"].startswith("unconnected-"): continue
    groups.setdefault((p["ref"], p["num"]), []).append(p)
for (ref, num), ps in groups.items():
    ps.sort(key=lambda p: -p["hx"] * p["hy"])
    keep = []
    for p in ps:
        if any(abs(p["x"] - q["x"]) < q["hx"] + p["hx"] and abs(p["y"] - q["y"]) < q["hy"] + p["hy"] for q in keep) and not p["th"]:
            continue
        keep.append(p)
    for p in keep:
        ex, ey = p["x"], p["y"]
        fine = ref in FINE and (FINE[ref] is None or num in FINE[ref])
        if fine:   # straight 0.2mm escape along the pad's long axis, away from the part
            cx, cy, _ = PLACE[ref]
            if ref == "J3": cx = cx - 10          # USB pads all escape towards the board (+x)
            if p["hx"] > p["hy"]:
                ex = p["x"] + math.copysign(p["hx"] + 0.5, p["x"] - cx); ex = round(ex / G) * G
            else:
                ey = p["y"] + math.copysign(p["hy"] + 0.5, p["y"] - cy); ey = round(ey / G) * G
            assert abs(p["x"] / G - round(p["x"] / G)) < 1e-6 or p["hx"] > p["hy"], (ref, num, p["x"])
            assert abs(p["y"] / G - round(p["y"] / G)) < 1e-6 or p["hy"] > p["hx"], (ref, num, p["y"])
            fixed.append((p["net"], 0, p["x"], p["y"], ex, ey, SIG))
        node = (int(round(ex / G)), int(round(ey / G)))
        nx, ny = node[0] * G, node[1] * G
        if not fine:
            off = math.hypot(nx - p["x"], ny - p["y"])
            assert off < min(p["hx"], p["hy"]), (ref, num, off)
            if off > 1e-4:
                for l in ((0, 1) if p["th"] else (0,)): fixed.append((p["net"], l, p["x"], p["y"], nx, ny, SIG))
        TERMS.setdefault(p["net"], []).append(dict(node=node, th=p["th"], ref=ref, num=num, fine=fine))
for (n, l, ax, ay, bx, by, hw) in fixed: base.seg(l, ax, ay, bx, by, hw, n)

padnode = {}
for n, ts in TERMS.items():
    for t in ts: padnode[t["node"]] = n


def via_ok(F, k, net):
    need = VIA_R + CL + M
    return not F.nv[k] and not F.blocked(0, k, net, need) and not F.blocked(1, k, net, need)


def astar(F, net, sources, goal_nodes, hw, to_plane=False, wt=1.3):
    """sources: set of (i, j, layer). Goal: a node in goal_nodes on a layer where it is reachable,
    or (to_plane) any spot on the bottom layer, which is the ground plane."""
    need = hw + CL + M; edge = hw + EDGE
    imin = int(math.ceil(edge / G)); imax = int((W - edge) / G); jmin = imin; jmax = int((H - edge) / G)
    tl = [(g[0], g[1]) for g in goal_nodes]
    if len(tl) > 12:
        sx = sum(s[0] for s in sources) / len(sources); sy = sum(s[1] for s in sources) / len(sources)
        tl = sorted(tl, key=lambda t: abs(t[0] - sx) + abs(t[1] - sy))[:12]
    def h(i, j):
        if to_plane or not tl: return 0.0
        best = INF
        for ti, tj in tl:
            dx = abs(ti - i); dy = abs(tj - j); v = (dx + dy) - 0.5858 * (dx if dx < dy else dy)
            if v < best: best = v
        return best * wt
    heap = []; g = {}; parent = {}
    for (i, j, l) in sources:
        s = (i, j, l, -1); g[s] = 0.0; heapq.heappush(heap, (h(i, j), 0.0, s))
    d1 = F.d1; n1 = F.n1; d2 = F.d2
    while heap:
        fcost, gc, s = heapq.heappop(heap)
        if gc > g.get(s, INF): continue
        i, j, l, d0 = s
        if ((i, j, l) in goal_nodes) or (to_plane and l == 1):
            path = [s]
            while s in parent: s = parent[s]; path.append(s)
            return path[::-1]
        lc = LAYER_COST[l]
        for d in range(8):
            if d0 >= 0:
                turn = (d - d0) % 8
                if turn > 4: turn = 8 - turn
                if turn > 2: continue
                tc = (0.0, 0.4, 1.5)[turn]
            else: tc = 0.0
            ni = i + DIRS[d][0]; nj = j + DIRS[d][1]
            if ni < imin or ni > imax or nj < jmin or nj > jmax: continue
            k = ni * NY + nj
            if (d1[l][k] < need and n1[l][k] != net) or d2[l][k] < need: continue
            ns = (ni, nj, l, d); ng = gc + STEP[d] * lc + tc
            if ng < g.get(ns, INF):
                g[ns] = ng; parent[ns] = s; heapq.heappush(heap, (ng + h(ni, nj), ng, ns))
        k = i * NY + j
        own_th = (i, j) in TH_NODES.get(net, ())
        if own_th or via_ok(F, k, net):
            ns = (i, j, 1 - l, -1); ng = gc + (0.05 if own_th else VIA_COST)
            if ng < g.get(ns, INF):
                g[ns] = ng; parent[ns] = s; heapq.heappush(heap, (ng + h(i, j), ng, ns))
    return None


TH_NODES = {}
for n, ts in TERMS.items():
    TH_NODES[n] = set(t["node"] for t in ts if t["th"])


def commit(F, net, path, hw, tracks, vias):
    run = [path[0]]; new_t = []; new_v = []
    def flush(run):
        if len(run) < 2: return
        a = run[0]; prev = run[0]; dprev = None
        for q in run[1:]:
            dcur = (q[0] - prev[0], q[1] - prev[1])
            if dprev is not None and dcur != dprev:
                new_t.append((net, a[2], a[0] * G, a[1] * G, prev[0] * G, prev[1] * G, hw)); a = prev
            dprev = dcur; prev = q
        new_t.append((net, a[2], a[0] * G, a[1] * G, prev[0] * G, prev[1] * G, hw))
    for s in path[1:]:
        if s[2] != run[-1][2]:
            flush(run)
            if (s[0], s[1]) not in TH_NODES.get(net, ()): new_v.append((net, s[0] * G, s[1] * G))
            run = [s]
        else: run.append(s)
    flush(run)
    for (n, l, ax, ay, bx, by, w) in new_t: F.seg(l, ax, ay, bx, by, w, n)
    for (n, x, y) in new_v:
        F.rect((0, 1), x, y, VIA_R, VIA_R, n, True)
        F.no_via_near(x, y, 0.9)            # keep the next via's hole clear of this one
    tracks += new_t; vias += new_v


def layers_of(t): return (0, 1) if t["th"] else (0,)


def route_net(F, net, hw, tracks, vias):
    ts = TERMS[net]
    # neck down to signal width when a power net has to reach a fine-pitch pad
    tree = set((ts[0]["node"][0], ts[0]["node"][1], l) for l in layers_of(ts[0]))
    todo = ts[1:]
    while todo:
        goals = set()
        for t in todo:
            for l in layers_of(t): goals.add((t["node"][0], t["node"][1], l))
        w = hw
        path = astar(F, net, tree, goals, w)
        if path is None and hw > SIG:
            w = SIG; path = astar(F, net, tree, goals, w)   # neck down where a power net meets a small pad
        if path is None: return False
        commit(F, net, path, w, tracks, vias)
        for s in path: tree.add((s[0], s[1], s[2]))
        end = (path[-1][0], path[-1][1])
        hit = [t for t in todo if t["node"] == end][0]
        for l in layers_of(hit): tree.add((end[0], end[1], l))
        todo = [t for t in todo if t is not hit]
    return True


def route_gnd(F, tracks, vias, only=None, skip=()):
    """Every surface-mount ground pad gets a short track to a via into the bottom ground plane,
    or to a ground through-hole pad if one is closer."""
    fails = []
    th_goals = set((t["node"][0], t["node"][1], 0) for t in TERMS["GND"] if t["th"])
    for t in sorted([t for t in TERMS["GND"] if not t["th"]], key=lambda t: (t["node"][0], t["node"][1])):
        key = (t["ref"], t["num"])
        if key in skip or (only is not None and key not in only): continue
        src = set([(t["node"][0], t["node"][1], 0)])
        w = 0.15
        path = astar(F, "GND", src, th_goals, w, to_plane=True)
        if path is None:
            w = SIG; path = astar(F, "GND", src, th_goals, w, to_plane=True)
        if path is None: fails.append(key); continue
        commit(F, "GND", path, w, tracks, vias)
    return fails


tracks = []; vias = []
if ROUTE:
    t0 = time.time()
    def span(net):
        ns = [t["node"] for t in TERMS[net]]
        return (max(n[0] for n in ns) - min(n[0] for n in ns)) + (max(n[1] for n in ns) - min(n[1] for n in ns))
    sig = sorted([n for n in TERMS if n not in POWER_NETS and n != "GND" and len(TERMS[n]) > 1], key=span)
    first = ["USB_DP", "USB_DM", "CC1", "CC2"]
    order = [n for n in POWER_NETS if n in TERMS] + first + [n for n in sig if n not in first]
    best = None; gnd_first = set()
    for attempt in range(10):
        F = base.copy(); tr = []; vi = []; failed = None
        gfail = route_gnd(F, tr, vi, only=gnd_first)       # ground pads that got boxed in on an earlier attempt
        for net in order:
            if not route_net(F, net, PWR if net in POWER_NETS else SIG, tr, vi):
                failed = net; break
        if failed is not None:
            print("attempt %d: could not route %s, moving it earlier" % (attempt + 1, failed))
            order.remove(failed); order.insert(0, failed); continue
        gfail += route_gnd(F, tr, vi, skip=gnd_first)
        if gfail and not set(gfail) <= gnd_first:
            print("attempt %d: ground pads boxed in: %s, giving them a via first" % (attempt + 1, gfail))
            gnd_first |= set(gfail); continue
        best = (tr, vi, gfail); break
    assert best is not None, "routing failed"
    tracks, vias, gfail = best
    if gfail: print("GROUND PADS WITHOUT A VIA:", gfail)
    print("routed %d nets in %.0fs: %d segments, %d vias" % (len(order) + 1, time.time() - t0, len(tracks), len(vias)))


def netobj(n): return nets[[k for k in nets if k.lstrip("/") == n][0]]


length = 0
for (n, l, ax, ay, bx, by, hw) in fixed + tracks:
    if not ROUTE and (n, l, ax, ay, bx, by, hw) not in fixed: continue
    t = pcbnew.PCB_TRACK(board); t.SetStart(V(ax, ay)); t.SetEnd(V(bx, by)); t.SetWidth(mm(hw * 2))
    t.SetLayer(pcbnew.F_Cu if l == 0 else pcbnew.B_Cu); t.SetNet(netobj(n))
    board.Add(t); length += math.hypot(bx - ax, by - ay)
for (n, x, y) in vias:
    v = pcbnew.PCB_VIA(board); v.SetPosition(V(x, y)); v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetDrill(mm(VIA_DRILL))
    try: v.SetWidth(pcbnew.F_Cu, mm(VIA_R * 2))
    except TypeError: v.SetWidth(mm(VIA_R * 2))
    v.SetNet(netobj(n)); board.Add(v)
if ROUTE: print("%.0f mm of track, %d on the bottom layer" % (length, sum(1 for t in tracks if t[1] == 1)))

# ---- GND pours on both layers
gnd = netobj("GND")
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(board); z.SetLayer(layer); z.SetNet(gnd)
    o = z.Outline(); o.NewOutline()
    for x, y in ((0, 0), (W, 0), (W, H), (0, H)): o.Append(mm(OX + x), mm(OY + y))
    z.SetLocalClearance(mm(0.25)); z.SetMinThickness(mm(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.25)); z.SetThermalReliefSpokeWidth(mm(0.3))
    z.SetAssignedPriority(0); z.SetZoneName("GND_F" if layer == pcbnew.F_Cu else "GND_B")
    board.Add(z)
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard("%s/%s.kicad_pcb" % (OUT, PROJECT), board)
print("saved board %.1f x %.1f mm, %d footprints" % (W, H, len(PARTS)))
