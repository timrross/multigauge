# Sets design rules, net classes and check severities in the KiCad project file.
# KiCad owns the file format, so this patches the file KiCad wrote instead of writing one from scratch.
import copy, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import OUT, PROJECT, POWER_NETS
path = "%s/%s.kicad_pro" % (OUT, PROJECT)
if not os.path.exists(path):
    json.dump({"meta": {"filename": PROJECT + ".kicad_pro", "version": 3}}, open(path, "w"), indent=2)
    print("project stub written"); sys.exit(0)
pro = json.load(open(path))
ns = pro.get("net_settings")
if not ns or not ns.get("classes"):
    print("project file not expanded by KiCad yet, nothing patched"); sys.exit(0)
default = [c for c in ns["classes"] if c["name"] == "Default"][0]
default.update(track_width=0.2, via_diameter=0.6, via_drill=0.3, clearance=0.15)
power = copy.deepcopy(default); power.update(name="Power", track_width=0.5, priority=0)
ns["classes"] = [default, power]
ns["netclass_patterns"] = [{"netclass": "Power", "pattern": p} for n in POWER_NETS for p in (n, "/" + n)]
ds = pro["board"]["design_settings"]
ds["track_widths"] = [0.0, 0.2, 0.3, 0.5]
ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3}]
# JLCPCB 2-layer limits are 0.127mm track and gap, 0.3mm hole, 0.6mm via. These rules sit inside them.
ds.setdefault("rules", {}).update(min_clearance=0.15, min_track_width=0.2, min_via_diameter=0.6,
                                  min_through_hole_diameter=0.3, min_copper_edge_clearance=0.3,
                                  min_hole_clearance=0.2, min_resolved_spokes=1, min_text_height=0.8)
# Connectors sit flush with the board edge on purpose, so their outline drawings cross it.
ds.setdefault("rule_severities", {})["silk_edge_clearance"] = "ignore"
json.dump(pro, open(path, "w"), indent=2)
print("project rules set: 0.2mm track, 0.15mm gap, 0.6/0.3mm via")
