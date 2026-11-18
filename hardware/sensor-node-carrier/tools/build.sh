#!/bin/sh
# Regenerates the whole KiCad project from tools/design.py and re-runs every check.
# WARNING: this overwrites the schematic, board, footprints and project file, including any hand edits.
T="$(cd "$(dirname "$0")" && pwd)"; P="$(dirname "$T")"; N=sensor-node-carrier
KP=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
TMP="$(mktemp -d)"; cd "$P" || exit 1
python3 "$T/gen_sch.py" || exit 1
kicad-cli sch upgrade $N.kicad_sch >/dev/null 2>&1
python3 "$T/gen_fp.py" && python3 "$T/gen_pro.py" || exit 1
kicad-cli sch erc --severity-all -o "$TMP/erc.rpt" $N.kicad_sch 2>&1 | grep "violations"
kicad-cli sch export netlist --format kicadsexpr -o "$TMP/sch.net" $N.kicad_sch >/dev/null 2>&1
python3 "$T/check_net.py" "$TMP/sch.net" || exit 1
"$KP" "$T/gen_pcb.py" "$TMP/sch.net" > "$TMP/pcb.log" 2>&1; RC=$?
grep -v "wxApp" "$TMP/pcb.log" | grep .
[ $RC -eq 0 ] || { echo "BOARD GENERATION FAILED: the board file was not updated"; exit 1; }
kicad-cli pcb drc --schematic-parity --severity-all --refill-zones --save-board -o "$TMP/drc.rpt" $N.kicad_pcb 2>&1 | grep "Found"
python3 "$T/gen_pro.py"
kicad-cli pcb drc --schematic-parity --severity-all -o "$TMP/drc2.rpt" $N.kicad_pcb 2>&1 | grep "Found"
# Manufacturing files: the zip in out/ is what gets uploaded to the board house.
rm -rf out/gerbers out/$N-gerbers.zip; mkdir -p out/gerbers
kicad-cli pcb export gerbers -o out/gerbers/ --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,Edge.Cuts --subtract-soldermask --check-zones $N.kicad_pcb >/dev/null 2>&1
kicad-cli pcb export drill -o out/gerbers/ --format excellon --excellon-separate-th --excellon-units mm $N.kicad_pcb >/dev/null 2>&1
(cd out/gerbers && zip -q ../$N-gerbers.zip *.g* *.drl) && echo "Gerbers: out/$N-gerbers.zip"
# Printouts: the schematic, and the board at 1:1 for the paper test fit (print at 100%, no "fit to page").
kicad-cli sch export pdf -o out/schematic.pdf $N.kicad_sch >/dev/null 2>&1 && echo "Schematic: out/schematic.pdf"
kicad-cli pcb export pdf --mode-single -l "F.Mask,F.SilkS,Edge.Cuts" --scale 1 --drill-shape-opt 2 --black-and-white -o out/paper-test-fit-1to1.pdf $N.kicad_pcb >/dev/null 2>&1 && echo "Paper fit: out/paper-test-fit-1to1.pdf"
