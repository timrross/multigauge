# Writes the two files JLCPCB's assembly service asks for, from design.py and KiCad's placement export:
#   out/jlcpcb-bom.csv   which part goes on which reference
#   out/jlcpcb-cpl.csv   where each part sits and how it is turned
# Usage: gen_jlc.py <kicad position csv>
import csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import OUT, PARTS, LCSC, HAND_FIT

def refkey(r):
    head = r.rstrip("0123456789"); return (head, int(r[len(head):]))

groups = {}
for ref in sorted(LCSC, key=refkey):
    value, sym, fp = PARTS[ref]
    groups.setdefault((LCSC[ref], value, fp.split(":")[1]), []).append(ref)
os.makedirs(OUT + "/out", exist_ok=True)
with open(OUT + "/out/jlcpcb-bom.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
    for (lcsc, value, fp), refs in sorted(groups.items(), key=lambda kv: refkey(kv[1][0])):
        w.writerow([value, ",".join(refs), fp, lcsc])

rows = list(csv.DictReader(open(sys.argv[1])))
placed = set()
with open(OUT + "/out/jlcpcb-cpl.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for r in sorted(rows, key=lambda r: refkey(r["Ref"])):
        if r["Ref"] not in LCSC: continue
        w.writerow([r["Ref"], "%.4fmm" % float(r["PosX"]), "%.4fmm" % float(r["PosY"]),
                    "Top" if r["Side"].lower().startswith("top") or r["Side"].lower() == "front" else "Bottom",
                    "%g" % (float(r["Rot"]) % 360)])
        placed.add(r["Ref"])
missing = set(LCSC) - placed
assert not missing, "no placement for %s" % sorted(missing)
print("JLCPCB files: %d part types, %d placements. Fit by hand: %s" % (len(groups), len(placed), ", ".join(sorted(HAND_FIT))))
