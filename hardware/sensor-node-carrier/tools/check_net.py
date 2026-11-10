import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design import NETS, NO_CONNECT, PARTS
from sexp import parse, find, get
t = parse(open(sys.argv[1]).read())
nets = {}
for n in find(find(t, 'nets')[0], 'net'):
    nets[get(n, 'name').lstrip('/')] = set((get(x, 'ref'), get(x, 'pin')) for x in find(n, 'node'))
comps = {get(c, 'ref'): get(c, 'footprint') for c in find(find(t, 'components')[0], 'comp')}
ok = True
want = {k: set(v) for k, v in NETS.items()}
for n in sorted(want):
    if nets.get(n) != want[n]:
        ok = False; print("MISMATCH", n, "want-only", want[n] - nets.get(n, set()), "got-only", nets.get(n, set()) - want[n])
ncs = set(NO_CONNECT); seen_nc = set()
for n, p in nets.items():
    if n in want: continue
    if n.startswith("unconnected-") and p <= ncs: seen_nc |= p
    else: ok = False; print("EXTRA NET", n, p)
if seen_nc != ncs: ok = False; print("NC DIFF", seen_nc ^ ncs)
if set(comps) != set(PARTS): ok = False; print("REF DIFF", set(comps) ^ set(PARTS))
for r, (v, s, fp) in PARTS.items():
    if comps.get(r) != fp: ok = False; print("FP DIFF", r, comps.get(r))
print("components: %d | named nets: %d of %d | no-connect pins: %d of %d" % (len(comps), len(set(nets) & set(want)), len(want), len(seen_nc), len(ncs)))
print("NETLIST MATCHES DESIGN" if ok else "NETLIST DOES NOT MATCH")
sys.exit(0 if ok else 1)
