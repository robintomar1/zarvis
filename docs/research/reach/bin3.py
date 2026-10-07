"""Bin reach WITH a Lift column (10x10 cm, directly behind arm footprint, floor to 1.1 m)."""
import sys, numpy as np
U = sys.argv[1]
src = open('bin.py').read(); src = src[:src.index('D = np.array')]
sys.argv = ['x', U, '5']
exec(src)
def boxes3(H, s, Hb):
    B = []; cr = -(s + 0.07) - 0.10; bf = min(-0.25, cr - 0.005)
    def add(xmin, xmax, ymin, ymax, zmin, zmax):
        ctr = np.array([(xmin + xmax) / 2 + s, (ymin + ymax) / 2, (zmin + zmax) / 2 - H])
        B.append(('o', coal.Box(xmax - xmin, ymax - ymin, zmax - zmin), coal.Transform3s(np.eye(3), ctr)))
    add(-2, 2, -2, 2, -0.2, 0.0)
    add(-0.45, -(s + 0.08), -0.225, 0.225, -0.2, Hb)
    add(cr, -(s + 0.07), -0.05, 0.05, 0.0, 1.10)               # Lift column
    add(-0.43, -0.425, -0.15, 0.15, Hb, Hb + WALL); add(bf - 0.005, bf, -0.15, 0.15, Hb, Hb + WALL)
    add(-0.43, bf, 0.145, 0.15, Hb, Hb + WALL); add(-0.43, bf, -0.15, -0.145, Hb, Hb + WALL)
    return B, bf
def coll3(q5, H, s, Hb):
    B, _ = boxes3(H, s, Hb)
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
def solve3(p, H, s, Hb, mode, n=80):
    def res(q):
        tcp, aa, _, _ = fk(q)
        extra = 0.05 * max(0, aa[2] + 0.5) if mode == 'tilt' else 0.1 * (aa - np.array([0, 0, -1.]))
        return np.r_[tcp - p, np.atleast_1d(extra)]
    for _ in range(n):
        r = least_squares(res, rng.uniform(lo, hi), bounds=(lo, hi), max_nfev=300)
        tcp, aa, _, _ = fk(r.x)
        okori = aa[2] <= -0.49 if mode == 'tilt' else aa[2] < -np.cos(np.radians(2))
        if np.linalg.norm(tcp - p) < 0.002 and okori and not coll3(r.x, H, s, Hb): return True
    return False
for s in (0.08, 0.12):
    for Hb in (0.25, 0.30):
        _, bf = boxes3(0.2, s, Hb); xs = np.round(np.linspace(bf - 0.04, -0.39, 3), 3)
        for H in (0.20, 0.25, 0.30):
            for mode in ('tilt', 'down'):
                for label, dz in (('bottom', 0.05), ('rim', WALL + 0.05)):
                    ok = [f"{x:+.2f}/{y:+.2f}" for x in xs for y in (0.0, 0.10, -0.10) if solve3(np.array([x + s, y, Hb + dz - H]), H, s, Hb, mode)]
                    print(f"s={s} Hb={Hb} H={H} {mode} {label}: {len(ok)}/9 {ok}", flush=True)
