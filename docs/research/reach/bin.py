"""Carry-Bin reach at the lowest Lift position. World: base front edge x=0, base 45x45 (x in [-0.45,0]),
arm base at (-s,0,H). Rear deck top at Hb; bin footprint x in [-0.43,-0.25], y +-0.15, walls 12 cm, 5 mm thick."""
import sys, numpy as np
U = sys.argv[1]; m_ = sys.argv[2]
sys.argv = ['x', U, '/dev/null', m_, '0.02', '0.12', 'mesh']
src = open('sweep.py').read(); src = src[:src.index('XS = ')]
exec(src)
WALL = 0.12
def bin_boxes(H, s, Hb):
    B = []
    def add(xmin, xmax, ymin, ymax, zmin, zmax):
        ctr = np.array([(xmin + xmax) / 2 + s, (ymin + ymax) / 2, (zmin + zmax) / 2 - H])
        B.append(('o', coal.Box(xmax - xmin, ymax - ymin, zmax - zmin), coal.Transform3s(np.eye(3), ctr)))
    add(-2, 2, -2, 2, -0.2, 0.0)                              # floor
    add(-0.45, -(s + 0.08), -0.225, 0.225, -0.2, Hb)          # rear deck (behind arm footprint)
    add(-0.43, -0.425, -0.15, 0.15, Hb, Hb + WALL)            # bin rear wall
    add(-0.255, -0.25, -0.15, 0.15, Hb, Hb + WALL)            # bin front wall
    add(-0.43, -0.25, 0.145, 0.15, Hb, Hb + WALL)             # side walls
    add(-0.43, -0.25, -0.15, -0.145, Hb, Hb + WALL)
    return B
def coll(q5, H, s, Hb):
    B = bin_boxes(H, s, Hb)
    for psi in np.linspace(-np.pi, np.pi, 9)[:-1]:
        q = np.zeros(m.nq); q[:5] = q5; q[5] = psi; q[6] = 0.02; q[7] = -0.02
        pin.updateGeometryPlacements(m, d, gm, gd, q)
        hit = False
        for i, g in enumerate(gm.geometryObjects):
            if g.parentJoint < 2: continue
            T = gd.oMg[i]; tf = coal.Transform3s(T.rotation, T.translation)
            for _, shp, bt in B:
                if coal.collide(g.geometry, tf, shp, bt, _req, coal.CollisionResult()): hit = True; break
            if hit: break
        if not hit: return False
    return True
def solve_bin(p, a, H, s, Hb, n=60):
    def res(q):
        tcp, aa, _, _ = fk(q); return np.r_[tcp - p, 0.1 * (aa - a)]
    kin = False
    for _ in range(n):
        r = least_squares(res, rng.uniform(lo, hi), bounds=(lo, hi), max_nfev=300)
        tcp, aa, _, _ = fk(r.x)
        if np.linalg.norm(tcp - p) < 0.002 and aa @ a > np.cos(np.radians(2)):
            kin = True
            if not coll(r.x, H, s, Hb): return 'ok', np.degrees(r.x).round(0)
    return ('kin-only' if kin else 'unreach'), None
D = np.array([0, 0, -1.])
for s in (0.08, 0.12, 0.16):
    for Hb in (0.25, 0.30):
        for H in (0.15, 0.20, 0.25, 0.30):
            row = []
            for label, dz in (('bottom', 0.05), ('rim', WALL + 0.05)):
                tally = []
                for x in (-0.29, -0.34, -0.39):
                    for y in (0.0, 0.10, -0.10):
                        st, q = solve_bin(np.array([x + s, y, Hb + dz - H]), D, H, s, Hb)
                        tally.append(st)
                row.append(f"{label}: {tally.count('ok')}/9 ok ({tally.count('kin-only')} kin-only)")
            print(f"s={s} Hb={Hb} H={H}: " + ' | '.join(row), flush=True)
