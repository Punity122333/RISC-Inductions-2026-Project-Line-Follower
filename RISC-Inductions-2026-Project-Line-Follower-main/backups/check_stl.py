import struct
import sys
from collections import defaultdict


def load(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:2000]:
        tris = []
        vals = []
        for line in data.decode("ascii", "replace").splitlines():
            tok = line.split()
            if tok and tok[0] == "vertex":
                vals.append([float(x) for x in tok[1:4]])
        for i in range(0, len(vals) - 2, 3):
            tris.append((tuple(vals[i]), tuple(vals[i + 1]), tuple(vals[i + 2])))
        return tris
    n = struct.unpack("<I", data[80:84])[0]
    tris = []
    off = 84
    for _ in range(n):
        v = struct.unpack("<12f", data[off:off + 48])
        tris.append((v[3:6], v[6:9], v[9:12]))
        off += 50
    return tris


def key(p, q=1e-4):
    return (round(p[0] / q), round(p[1] / q), round(p[2] / q))


def analyze(path):
    tris = load(path)
    verts = {}
    coords = {}
    clist = []
    faces = []
    for t in tris:
        f = []
        for p in t:
            k = key(p)
            if k not in verts:
                verts[k] = len(verts)
                coords[k] = p
                clist.append(p)
            f.append(verts[k])
        faces.append(tuple(f))
    directed = defaultdict(int)
    for a, b, c in faces:
        for e in ((a, b), (b, c), (c, a)):
            directed[e] += 1
    edges = defaultdict(int)
    for e, n in directed.items():
        edges[(min(e), max(e))] += n
    bad = [e for e, n in edges.items() if n != 2]
    unpaired = set()
    for (a, b), n in directed.items():
        if directed.get((b, a), 0) != n:
            unpaired.add((a, b))
    adj = defaultdict(set)
    for a, b, c in faces:
        adj[a].update((b, c))
        adj[b].update((a, c))
        adj[c].update((a, b))
    seen = set()
    comps = []
    for v in range(len(verts)):
        if v in seen:
            continue
        stack = [v]
        seen.add(v)
        comp = []
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for nb in adj[cur]:
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        comps.append(comp)
    xs = [p[0] for p in clist]
    ys = [p[1] for p in clist]
    zs = [p[2] for p in clist]
    vol = 0.0
    for (pa, pb, pc) in tris:
        vol += (
            pa[0] * (pb[1] * pc[2] - pb[2] * pc[1])
            - pa[1] * (pb[0] * pc[2] - pb[2] * pc[0])
            + pa[2] * (pb[0] * pc[1] - pb[1] * pc[0])
        ) / 6.0
    print("file      :", path)
    print("triangles :", len(tris))
    print("vertices  :", len(verts))
    print("bbox X    : %.3f .. %.3f  (%.3f)" % (min(xs), max(xs), max(xs) - min(xs)))
    print("bbox Y    : %.3f .. %.3f  (%.3f)" % (min(ys), max(ys), max(ys) - min(ys)))
    print("bbox Z    : %.3f .. %.3f  (%.3f)" % (min(zs), max(zs), max(zs) - min(zs)))
    print("volume mm3: %.1f" % vol)
    print("nonmanifold edges:", len(bad))
    print("unpaired edges   :", len(unpaired))
    print("components :", len(comps))
    for i, c in enumerate(sorted(comps, key=len, reverse=True)[:6]):
        pts = [clist[v] for v in c]
        print(
            "  comp%d n=%-6d x %8.2f..%8.2f y %8.2f..%8.2f z %8.2f..%8.2f"
            % (
                i,
                len(c),
                min(p[0] for p in pts),
                max(p[0] for p in pts),
                min(p[1] for p in pts),
                max(p[1] for p in pts),
                min(p[2] for p in pts),
                max(p[2] for p in pts),
            )
        )
    return len(bad) == 0 and len(unpaired) == 0 and len(comps) == 1


if __name__ == "__main__":
    ok = True
    for p in sys.argv[1:]:
        ok = analyze(p) and ok
        print()
    sys.exit(0 if ok else 1)
