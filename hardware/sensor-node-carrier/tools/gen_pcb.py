# Generates sensor-node-carrier.kicad_pcb: placement, outline, silk, routing, GND pours.
# Run with KiCad's bundled python (needs pcbnew).
import os, sys, math, heapq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcbnew
from design import *
from sexp import parse, find, get

ROUTE = "--no-route" not in sys.argv
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
NETFILE = sys.argv[1]
W, H = 85.09, 66.04          # board size, mm
OX, OY = 101.6, 63.5         # board origin on the page
mm = pcbnew.FromMM; tomm = pcbnew.ToMM


def V(x, y): return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


# ref: (pad 1 x, pad 1 y, rotation)
PLACE = {
    "U1": (31.75, 2.54, 0), "U2": (19.05, 49.53, 90), "U3": (63.5, 30.48, 270), "U4": (59.69, 12.7, 270),
    "J1": (13.97, 59.69, 0), "J2": (6.35, 12.7, 270), "J3": (64.77, 50.8, 90), "J4": (55.88, 27.94, 0),
    "L1": (10.16, 3.81, 0), "C1": (16.51, 12.7, 0), "C2": (25.4, 10.16, 270), "D1": (25.4, 20.32, 180),
    "C6": (26.67, 27.94, 0), "C5": (26.67, 31.75, 0), "C4": (26.67, 35.56, 0), "C3": (26.67, 39.37, 0),
    "TP7": (35.56, 27.94, 0), "TP6": (35.56, 31.75, 0), "TP5": (35.56, 35.56, 0), "TP4": (35.56, 39.37, 0),
    "R8": (50.8, 27.94, 180), "R6": (40.64, 31.75, 0), "R7": (50.8, 35.56, 180), "R5": (40.64, 39.37, 0),
    "R2": (36.83, 43.18, 180), "R1": (26.67, 46.99, 0), "R3": (26.67, 50.8, 0), "R4": (40.64, 50.8, 0),
    "TP1": (55.88, 41.91, 0), "TP2": (55.88, 45.72, 0), "TP3": (55.88, 49.53, 0),
    "H1": (3.81, 3.81, 0), "H2": (81.28, 3.81, 0), "H3": (3.81, 62.23, 0), "H4": (81.28, 62.23, 0),
}
TP_LABEL = {"TP1": "5V", "TP2": "3V3", "TP3": "GND", "TP4": "A0", "TP5": "A1", "TP6": "A2", "TP7": "A3"}
J1_LABEL = ["5V", "GND", "OILP", "5V", "GND", "BST", "OILT", "GND", "IAT", "GND", "SP1", "SP2"]

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


def text(s, x, y, size=1.0, rot=0, just=None, layer=pcbnew.F_SilkS):
    t = pcbnew.PCB_TEXT(board); t.SetText(s); t.SetPosition(V(x, y)); t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(size * 0.15))
    if rot: t.SetTextAngleDegrees(rot)
    if just == "left": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    if just == "right": t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
    board.Add(t)


# ---- outline
e = pcbnew.PCB_SHAPE(board); e.SetShape(pcbnew.SHAPE_T_RECT); e.SetStart(V(0, 0)); e.SetEnd(V(W, H))
e.SetLayer(pcbnew.Edge_Cuts); e.SetWidth(mm(0.1)); board.Add(e)

# ---- footprints
PADS = []   # dicts: ref, num, net, x, y (board-local mm), shape, r / rect
for ref, (value, sym, fpid) in PARTS.items():
    lib, name = fpid.split(":")
    libdir = (OUT + "/SensorNode.pretty") if lib == "SensorNode" else "%s/%s.pretty" % (FPDIR, lib)
    fp = pcbnew.FootprintLoad(libdir, name)
    assert fp is not None, fpid
    px, py, rot = PLACE[ref]
    fp.SetReference(ref); fp.SetValue(value); fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetPosition(V(0, 0)); fp.SetOrientationDegrees(rot)
    pads = list(fp.Pads())
    p1 = [p for p in pads if p.GetNumber() in ("1", "")][0].GetPosition()
    fp.Move(pcbnew.VECTOR2I(mm(OX + px) - p1.x, mm(OY + py) - p1.y))
    fp.SetPath(pcbnew.KIID_PATH("/" + uid("sym:" + ref)))
    fp.SetSheetname("/"); fp.SetSheetfile(PROJECT + ".kicad_sch")
    board.Add(fp)
    kind = ref.rstrip("0123456789")
    if kind in ("R", "C", "D", "L"):       # put the reference on the part body
        bb = [p.GetPosition() for p in pads]
        fp.Reference().SetPosition(pcbnew.VECTOR2I((bb[0].x + bb[-1].x) // 2, (bb[0].y + bb[-1].y) // 2))
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8))); fp.Reference().SetTextThickness(mm(0.12))
    if kind in ("TP", "H") or ref == "J1":
        fp.Reference().SetVisible(False)
    if ref == "C1":
        fp.Reference().SetPosition(V(21.9, 15.6)); fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8))); fp.Reference().SetTextThickness(mm(0.12))
    if kind == "TP":
        text(TP_LABEL[ref], px + 1.5, py, 0.8, just="left")
    for p in pads:
        num = p.GetNumber(); pos = p.GetPosition()
        x = tomm(pos.x) - OX; y = tomm(pos.y) - OY
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            PADS.append(dict(ref=ref, num=num, net="__hole__", x=x, y=y, shape="c", r=3.3)); continue
        net = padnet[(ref, num)]
        p.SetNet(nets[net])
        net = net.lstrip("/")
        bb = p.GetBoundingBox()
        sx = tomm(bb.GetWidth()); sy = tomm(bb.GetHeight())
        if num == "1" and kind in ("U", "J", "C", "D"):   # square / rounded-square pin 1 pads
            PADS.append(dict(ref=ref, num=num, net=net, x=x, y=y, shape="r", hx=sx / 2, hy=sy / 2))
        else:
            PADS.append(dict(ref=ref, num=num, net=net, x=x, y=y, shape="c", r=max(sx, sy) / 2))

# ---- board silk
for i, s in enumerate(J1_LABEL):
    text(s, 13.97 + i * 5.08, 53.45, 0.8)
text("+5V", 12.95, 12.7, 0.8, rot=90); text("GND", 12.95, 17.78, 0.8, rot=90)
text("MultiGauge", 46.0, 42.6, 0.8); text("sensor node", 46.0, 44.3, 0.8); text("rev 2  2026-11", 46.0, 46.0, 0.8)
# J4 labels sit on the left of the header: the MAX31855 pin names take the space on its right.
for i, s in enumerate(("3V3", "GND", "SDA", "SCL")): text(s, 54.2, 27.94 + i * 2.54, 0.8, just="right")
text("J1", 7.9, 56.4, 0.8); text("LOOM", 7.9, 57.8, 0.8)

# ============================================================ router
P = 0.635
NX = int(round(W / P)) + 1; NY = int(round(H / P)) + 1
CL, M = 0.2, 0.2          # design clearance, extra margin so segments between nodes stay legal
VIA_R, VIA_DRILL = 0.4, 0.4
LAYER_COST = [1.2, 1.0]   # F.Cu, B.Cu  (prefer the back so the front pour stays whole)
VIA_COST = 14.0
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


def pad_dist(p, x, y):
    if p["shape"] == "c": return math.hypot(x - p["x"], y - p["y"]) - p["r"]
    dx = max(abs(x - p["x"]) - p["hx"], 0); dy = max(abs(y - p["y"]) - p["hy"], 0)
    return math.hypot(dx, dy)


static = {}
for p in PADS:
    reach = (p["r"] if p["shape"] == "c" else max(p["hx"], p["hy"]) * 1.5) + 1.4
    for i in range(max(0, int((p["x"] - reach) / P)), min(NX - 1, int((p["x"] + reach) / P) + 1) + 1):
        for j in range(max(0, int((p["y"] - reach) / P)), min(NY - 1, int((p["y"] + reach) / P) + 1) + 1):
            d = pad_dist(p, i * P, j * P)
            if d < 1.4: static.setdefault((i, j), []).append((d, p["net"]))
dyn = ({}, {})
padnode = {}
for p in PADS:
    if p["net"] == "__hole__": continue
    i = int(round(p["x"] / P)); j = int(round(p["y"] / P))
    p["node"] = (i, j); padnode.setdefault((i, j), p["net"])


def seg_dist(x, y, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay; L2 = vx * vx + vy * vy
    t = 0 if L2 == 0 else max(0, min(1, ((x - ax) * vx + (y - ay) * vy) / L2))
    return math.hypot(x - ax - t * vx, y - ay - t * vy)


def add_obstacle(layer, ax, ay, bx, by, hw, net):
    for i in range(max(0, int((min(ax, bx) - hw - 1.4) / P)), min(NX - 1, int((max(ax, bx) + hw + 1.4) / P) + 1) + 1):
        for j in range(max(0, int((min(ay, by) - hw - 1.4) / P)), min(NY - 1, int((max(ay, by) + hw + 1.4) / P) + 1) + 1):
            d = seg_dist(i * P, j * P, ax, ay, bx, by) - hw
            if d < 1.4: dyn[layer].setdefault((i, j), []).append((d, net))


def blocked(i, j, layer, net, need):
    for d, n in static.get((i, j), ()):
        if n != net and d < need: return True
    for d, n in dyn[layer].get((i, j), ()):
        if n != net and d < need: return True
    return False


def via_ok(i, j, net):
    for d, n in static.get((i, j), ()):
        if d < (VIA_R + CL + M if n != net else VIA_R + 0.3): return False
    return not (blocked(i, j, 0, net, VIA_R + CL + M) or blocked(i, j, 1, net, VIA_R + CL + M))


def astar(net, sources, targets, hw):
    need = hw + CL + M; edge = hw + 0.7
    imin = int(math.ceil(edge / P)); imax = int((W - edge) / P); jmin = imin; jmax = int((H - edge) / P)
    tl = list(targets)
    def h(i, j):
        best = 1e9
        for ti, tj in tl:
            dx = abs(ti - i); dy = abs(tj - j); v = max(dx, dy) + 0.4142 * min(dx, dy)
            if v < best: best = v
        return best
    heap = []; g = {}; parent = {}
    for (i, j, l) in sources:
        s = (i, j, l, -1); g[s] = 0; heapq.heappush(heap, (h(i, j), 0, s))
    while heap:
        f, gc, s = heapq.heappop(heap)
        if gc > g.get(s, 1e18): continue
        i, j, l, d0 = s
        if (i, j) in targets:
            path = [s]
            while s in parent: s = parent[s]; path.append(s)
            return path[::-1]
        for d in range(8):
            if d0 >= 0:
                turn = min((d - d0) % 8, (d0 - d) % 8)
                if turn > 2: continue
                tc = (0, 0.3, 1.0)[turn]
            else: tc = 0
            di, dj = DIRS[d]; ni, nj = i + di, j + dj
            if not (imin <= ni <= imax and jmin <= nj <= jmax): continue
            if blocked(ni, nj, l, net, need): continue
            ns = (ni, nj, l, d); ng = gc + (1.4142 if di and dj else 1.0) * LAYER_COST[l] + tc
            if ng < g.get(ns, 1e18):
                g[ns] = ng; parent[ns] = s; heapq.heappush(heap, (ng + h(ni, nj), ng, ns))
        own_pad = padnode.get((i, j)) == net
        if own_pad or via_ok(i, j, net):
            ns = (i, j, 1 - l, -1); ng = gc + (0.05 if own_pad else VIA_COST)
            if not blocked(i, j, 1 - l, net, need) and ng < g.get(ns, 1e18):
                g[ns] = ng; parent[ns] = s; heapq.heappush(heap, (ng + h(i, j), ng, ns))
    return None


tracks = []; vias = []


def netobj(n): return nets[[k for k in nets if k.lstrip("/") == n][0]]


def commit(net, path, hw):
    """Turn a node path into merged segments / vias and register them as obstacles."""
    run = [path[0]]
    def flush(run):
        if len(run) < 2: return
        a = run[0]; prev = run[0]; dprev = None
        for q in run[1:]:
            dcur = (q[0] - prev[0], q[1] - prev[1])
            if dprev is not None and dcur != dprev:
                tracks.append((net, a[2], a[0] * P, a[1] * P, prev[0] * P, prev[1] * P, hw)); a = prev
            dprev = dcur; prev = q
        tracks.append((net, a[2], a[0] * P, a[1] * P, prev[0] * P, prev[1] * P, hw))
    for s in path[1:]:
        if s[2] != run[-1][2]:
            flush(run)
            if padnode.get((s[0], s[1])) != net: vias.append((net, s[0] * P, s[1] * P))
            run = [s]
        else: run.append(s)
    flush(run)


def route_net(net, hw):
    pads = [p for p in PADS if p["net"] == net]
    for p in pads: assert abs(p["x"] - p["node"][0] * P) < 0.02 and abs(p["y"] - p["node"][1] * P) < 0.02, p
    tree = set((pads[0]["node"][0], pads[0]["node"][1], l) for l in (0, 1))
    todo = set(p["node"] for p in pads[1:]) - set([pads[0]["node"]])
    n0 = len(tracks); v0 = len(vias)
    while todo:
        path = astar(net, tree, todo, hw)
        if path is None: return False
        commit(net, path, hw)
        for s in path: tree.add((s[0], s[1], s[2]))
        end = (path[-1][0], path[-1][1]); todo.discard(end); tree.add((end[0], end[1], 0)); tree.add((end[0], end[1], 1))
    for (n, l, ax, ay, bx, by, w) in tracks[n0:]: add_obstacle(l, ax, ay, bx, by, w, n)
    for (n, x, y) in vias[v0:]:
        add_obstacle(0, x, y, x, y, VIA_R, n); add_obstacle(1, x, y, x, y, VIA_R, n)
    return True


if ROUTE:
    def span(net):
        ps = [p for p in PADS if p["net"] == net]
        return (max(p["x"] for p in ps) - min(p["x"] for p in ps)) + (max(p["y"] for p in ps) - min(p["y"] for p in ps))
    base = sorted([n for n in NETS if n not in POWER_NETS and n != "GND"], key=span)
    best = None
    for vc in (14.0, 30.0, 60.0):
        for lc in ([1.2, 1.0], [1.0, 1.0], [1.0, 1.2]):
            for sig in (base, base[::-1]):
                VIA_COST = vc; LAYER_COST[:] = lc
                order = ["+5V_MCU", "5V_IN", "+5V", "+3V3"] + list(sig)
                for attempt in range(4):
                    tracks[:] = []; vias[:] = []; dyn[0].clear(); dyn[1].clear(); failed = None
                    for net in order:
                        if not route_net(net, 0.5 if net in POWER_NETS else 0.25):
                            failed = net; break
                    if failed is None: break
                    order.remove(failed); order.insert(0, failed)
                if failed is not None: continue
                ln = sum(math.hypot(q[4] - q[2], q[5] - q[3]) for q in tracks)
                score = len(vias) * 25 + ln
                if best is None or score < best[0]: best = (score, list(tracks), list(vias), vc, list(lc), list(order))
    assert best is not None, "routing failed"
    tracks[:] = best[1]; vias[:] = best[2]; order = best[5]
    print("best variant: via cost %g, layer cost %s" % (best[3], best[4]))
    length = 0
    for (n, l, ax, ay, bx, by, hw) in tracks:
        t = pcbnew.PCB_TRACK(board); t.SetStart(V(ax, ay)); t.SetEnd(V(bx, by)); t.SetWidth(mm(hw * 2))
        t.SetLayer(pcbnew.F_Cu if l == 0 else pcbnew.B_Cu); t.SetNet(netobj(n))
        board.Add(t); length += math.hypot(bx - ax, by - ay)
    for (n, x, y) in vias:
        v = pcbnew.PCB_VIA(board); v.SetPosition(V(x, y)); v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetDrill(mm(VIA_DRILL))
        try: v.SetWidth(pcbnew.F_Cu, mm(VIA_R * 2))
        except TypeError: v.SetWidth(mm(VIA_R * 2))
        v.SetNet(netobj(n)); board.Add(v)
    print("routed %d nets: %d segments, %d vias, %.0f mm of track (%d on F.Cu)" % (
        len(order), len(tracks), len(vias), length, sum(1 for t in tracks if t[1] == 0)))

# ---- GND pours on both layers
gnd = netobj("GND")
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(board); z.SetLayer(layer); z.SetNet(gnd)
    o = z.Outline(); o.NewOutline()
    for x, y in ((0, 0), (W, 0), (W, H), (0, H)): o.Append(mm(OX + x), mm(OY + y))
    z.SetLocalClearance(mm(0.3)); z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.4)); z.SetThermalReliefSpokeWidth(mm(0.6))
    z.SetAssignedPriority(0); z.SetZoneName("GND_F" if layer == pcbnew.F_Cu else "GND_B")
    board.Add(z)
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard("%s/%s.kicad_pcb" % (OUT, PROJECT), board)
print("saved board %.2f x %.2f mm, %d footprints" % (W, H, len(PARTS)))
