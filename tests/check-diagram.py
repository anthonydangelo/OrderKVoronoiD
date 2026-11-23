#!/usr/bin/env python3
# Checks OrderKVoronoiD outputs against a brute-force computation of the k nearest
# sites, for every case in tests/cases.txt that asks for a diagram (the error cases
# and -h are skipped).
#
# Usage (from the repo root):
#   python3 tests/check-diagram.py <label>          checks tests/out/<label>/
#
# For each case:
# - on a tie: at the midpoint of each output edge, the k-th and (k+1)-th nearest sites
#   must be equidistant, since edges are where the set of k nearest sites changes;
# - coverage: on a 60 x 60 grid over the output's bounding box, wherever two neighbouring
#   grid points have different sets of k nearest sites, an output edge must cross (or
#   pass within the tolerance of) the segment between them; this catches missing edges;
# - zero-length edges (both endpoints the same point) and repeated edges are counted.
# Repeated input points are counted once, as the program does.
# The output prints 6 significant digits, so the tolerance is 1e-5 times the largest
# coordinate. A grid point that lies almost exactly on an edge can still be counted as
# uncovered now and then; look at such a case before concluding it is wrong.
# Exit code: 0 if every case is OK.
import json, math, os, re, sys

def parse_args(argstr):
    m = re.search(r'-i (\S+)', argstr)
    pts = json.loads(m.group(1)) if m else []
    k = int(re.search(r'-o (\d+)', argstr).group(1))
    # The program removes exact duplicates (keeping the first copy), so do the same.
    sites = []
    for p in pts:
        p = tuple(map(float, p))
        if p not in sites:
            sites.append(p)
    return sites, k

def dists(sites, q):
    return sorted(math.hypot(q[0] - s[0], q[1] - s[1]) for s in sites)

def kset(sites, q, k):
    order = sorted(range(len(sites)),
                   key=lambda i: (math.hypot(q[0] - sites[i][0], q[1] - sites[i][1]), i))
    return frozenset(order[:k])

def point_seg_dist(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L = vx * vx + vy * vy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)

def segs_close(p1, p2, q1, q2, tol):
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if (orient(q1, q2, p1) * orient(q1, q2, p2) <= 0
            and orient(p1, p2, q1) * orient(p1, p2, q2) <= 0):
        return True
    return min(point_seg_dist(p1, q1, q2), point_seg_dist(p2, q1, q2),
               point_seg_dist(q1, p1, p2), point_seg_dist(q2, p1, p2)) <= tol

def check(args, out_file, grid=60):
    sites, k = parse_args(args)
    vd = json.load(open(out_file))['voronoiDiagram']
    P = [(float(p['x']), float(p['y'])) for p in vd['voronoiPoints']]
    E = [(int(e['voronoiEdge']['firstIndex']), int(e['voronoiEdge']['secondIndex']))
         for e in vd['voronoiEdgesPtIndices']]
    tol = 1e-5 * max([1.0] + [abs(c) for p in P for c in p])
    off_tie = 0
    for a, b in E:
        d = dists(sites, ((P[a][0] + P[b][0]) / 2, (P[a][1] + P[b][1]) / 2))
        if k < len(d) and d[k] - d[k - 1] > tol:
            off_tie += 1
    zero = sum(1 for a, b in E if a == b)
    repeated = len(E) - len(set(tuple(sorted(e)) for e in E))
    uncovered = checked = 0
    if P:
        x0, x1 = min(p[0] for p in P), max(p[0] for p in P)
        y0, y1 = min(p[1] for p in P), max(p[1] for p in P)
        if x1 > x0 and y1 > y0:
            G = [[(x0 + (i + 0.5) * (x1 - x0) / grid, y0 + (j + 0.5) * (y1 - y0) / grid)
                  for j in range(grid)] for i in range(grid)]
            S = [[kset(sites, G[i][j], k) for j in range(grid)] for i in range(grid)]
            segs = [(P[a], P[b]) for a, b in E]
            for i in range(grid):
                for j in range(grid):
                    for ii, jj in ((i + 1, j), (i, j + 1)):
                        if ii < grid and jj < grid and S[i][j] != S[ii][jj]:
                            checked += 1
                            if not any(segs_close(G[i][j], G[ii][jj], s, t, tol) for s, t in segs):
                                uncovered += 1
    return len(E), off_tie, checked, uncovered, zero, repeated

def main():
    if len(sys.argv) != 2:
        sys.exit('usage: python3 tests/check-diagram.py <label>')
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(tests_dir, 'out', sys.argv[1])
    problems = 0
    for line in open(os.path.join(tests_dir, 'cases.txt')):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, desc, args = line.split('|', 2)
        # Only cases that ask for a diagram: the error cases and -h produce none.
        if name.startswith('err') or not (re.search(r'-i \S', args) and re.search(r'-o \d', args)):
            continue
        try:
            n, off_tie, checked, uncovered, zero, repeated = check(args, os.path.join(outdir, name + '.json'))
        except (IOError, ValueError, KeyError) as e:
            print('{:16} ERROR ({}: {})'.format(name, e.__class__.__name__, e))
            problems += 1
            continue
        ok = not (off_tie or uncovered or zero or repeated)
        problems += 0 if ok else 1
        print('{:16} {:8} edges {:4}, not on a tie {:3}, uncovered {:3} of {:4}, zero-length {}, repeated {}'.format(
            name, 'OK' if ok else 'PROBLEM', n, off_tie, uncovered, checked, zero, repeated))
    print('{} case(s) with problems.'.format(problems))
    sys.exit(1 if problems else 0)

if __name__ == '__main__':
    main()
