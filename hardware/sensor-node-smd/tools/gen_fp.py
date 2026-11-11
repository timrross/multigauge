# Writes the project footprint library.
# One footprint: the ESP32-C3-WROOM-02U from KiCad's library with the thermal vias under its ground pad
# opened from 0.2mm to 0.3mm holes, which is JLCPCB's standard minimum for a 2-layer board.
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import OUT
SRC = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/RF_Module.pretty/ESP32-C3-WROOM-02U.kicad_mod"
NAME = "ESP32-C3-WROOM-02U_Via0.3mm"
LIB = OUT + "/SensorNodeSMD.pretty"
os.makedirs(LIB, exist_ok=True)
t = open(SRC).read()
n = t.count("(drill 0.2)")
assert n > 0 and t.startswith('(footprint "ESP32-C3-WROOM-02U"')
t = t.replace("(drill 0.2)", "(drill 0.3)").replace('(footprint "ESP32-C3-WROOM-02U"', '(footprint "%s"' % NAME, 1)
open("%s/%s.kicad_mod" % (LIB, NAME), "w").write(t)
open(OUT + "/fp-lib-table", "w").write(
    '(fp_lib_table\n\t(version 7)\n\t(lib (name "SensorNodeSMD")(type "KiCad")(uri "${KIPRJMOD}/SensorNodeSMD.pretty")(options "")(descr "Sensor node SMD footprints"))\n)\n')
print("footprint library written (%d thermal vias opened to 0.3mm)" % n)
