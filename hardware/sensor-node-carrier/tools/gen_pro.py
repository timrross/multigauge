# Sets net classes and track / via presets in the KiCad project file.
# KiCad owns the file format, so this patches the file KiCad wrote instead of writing one from scratch.
import copy, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import OUT, PROJECT
path = "%s/%s.kicad_pro" % (OUT, PROJECT)
if not os.path.exists(path):
    json.dump({"meta": {"filename": PROJECT + ".kicad_pro", "version": 3}}, open(path, "w"), indent=2)
    print("project stub written"); sys.exit(0)
pro = json.load(open(path))
ns = pro.get("net_settings")
if not ns or not ns.get("classes"):
    print("project file not expanded by KiCad yet, nothing patched"); sys.exit(0)
default = [c for c in ns["classes"] if c["name"] == "Default"][0]
default.update(track_width=0.5, via_diameter=0.8, via_drill=0.4, clearance=0.2)
power = copy.deepcopy(default); power.update(name="Power", track_width=1.0, priority=0)
ns["classes"] = [default, power]
ns["netclass_patterns"] = [{"netclass": "Power", "pattern": p} for p in ("+5V", "+5V_MCU", "+3V3", "/5V_IN")]
ds = pro["board"]["design_settings"]
ds["track_widths"] = [0.0, 0.5, 1.0]
ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.8, "drill": 0.4}]
ds.setdefault("rules", {}).update(min_clearance=0.2, min_track_width=0.25, min_via_diameter=0.6)
json.dump(pro, open(path, "w"), indent=2)
print("project net classes set: Default 0.5mm, Power 1.0mm")
