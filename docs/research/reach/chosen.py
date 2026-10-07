"""Reachability for the chosen Zarvis geometry (front-face Lift). Usage:
python -I chosen.py URDF_ABS s task   (task in floor|counter|bins|stow)
World: floor z=0, base footprint x in [-0.45,0], y in [-0.225,0.225]; arm axis at (xa=-s, 0, H).
Obstacles: chassis x[-0.45,0] z<=0.18 (notched under carriage); deck x[-0.45, xa-0.07] z<=Hb;
column x[xa-0.17, xa-0.07] y+-0.05 z<=1.1; bins (two 15x25 compartments, 12 cm walls) beside column."""
import sys, json, numpy as np
U, S_, TASK = sys.argv[1], float(sys.argv[2]), sys.argv[3]
sys.argv = ['x', U, '/dev/null', '5', '0.02', '0.12', 'mesh']
src = open('sweep.py').read(); src = src[:src.index('XS = ')]
exec(src)
xa = -S_
R70 = 0.70 * 0.767
WALL = 0.12
BIN_X = (xa - 0.08 - 0.25, xa - 0.08)
BIN_Y = {'L': (0.06, 0.21), 'R': (-0.21, -0.06)}

def world_boxes(Hb=0.30, counter_edge=None, bins=True):
    W = [('floor', (-2, 2, -2, 2, -0.2, 0.0)),
         ('chassis', (-0.45, 0.0, -0.225, 0.225, -0.2, 0.18)),
         ('deck', (-0.45, xa - 0.07, -0.225, 0.225, 0.18, Hb)),
         ('column', (xa - 0.17, xa - 0.07, -0.05, 0.05, 0.0, 1.10))]
    if bins:
        x0, x1 = BIN_X
        for k, (y0, y1) in BIN_Y.items():
            W += [(f'bin{k}', (x0, x0 + 0.005, y0, y1, Hb, Hb + WALL)), (f'bin{k}', (x1 - 0.005, x1, y0, y1, Hb, Hb + WALL)),
                  (f'bin{k}', (x0, x1, y0, y0 + 0.005, Hb, Hb + WALL)), (f'bin{k}', (x0, x1, y1 - 0.005, y1, Hb, Hb + WALL))]
    if counter_edge is not None:
        c = counter_edge
        W += [('counter', (c, c + 0.6, -1, 1, 0.86, 0.90)), ('cabinet', (c + 0.03, c + 0.6, -1, 1, 0.18, 0.86))]
    return W

def to_arm(W, H):
    out = []
    for name, (x0, x1, y0, y1, z0, z1) in W:
        ctr = np.array([(x0 + x1) / 2 - xa, (y0 + y1) / 2, (z0 + z1) / 2 - H])
        out.append((name, coal.Box(x1 - x0, y1 - y0, z1 - z0), coal.Transform3s(np.eye(3), ctr)))
    return out

def static_hits(B):
    """base_link (fixed to carriage) vs obstacles other than column/deck/chassis it is mounted against."""
    q = np.zeros(m.nq); pin.updateGeometryPlacements(m, d, gm, gd, q)
    hits = []
    for i, g in enumerate(gm.geometryObjects):
        if g.parentJoint != 0: continue
        T = gd.oMg[i]; tf = coal.Transform3s(T.rotation, T.translation)
        for name, shp, bt in B:
            if name in ('column', 'deck', 'chassis'): continue
            if coal.collide(g.geometry, tf, shp, bt, _req, coal.CollisionResult()): hits.append(name)
    return hits

def coll(q5, B, psis=None):
    for psi in (np.linspace(-np.pi, np.pi, 9)[:-1] if psis is None else psis):
        q = np.zeros(m.nq); q[:5] = q5; q[5] = psi; q[6] = 0.02; q[7] = -0.02
        pin.updateGeometryPlacements(m, d, gm, gd, q)
        hit = None
        for i, g in enumerate(gm.geometryObjects):
            if g.parentJoint < 1: continue
            T = gd.oMg[i]; tf = coal.Transform3s(T.rotation, T.translation)
            for name, shp, bt in B:
                if name == 'floor' and 'finger' in g.name: continue
                if g.parentJoint == 1 and name in ('column', 'chassis'): continue  # link1 sits on the carriage
                if coal.collide(g.geometry, tf, shp, bt, _req, coal.CollisionResult()): hit = name; break
            if hit: break
        if hit is None: return None, psi
    return hit, None

def ik(p, mode, B, n=50):
    def res(q):
        tcp, a, _, _ = fk(q)
        if mode == 'down': o = 0.1 * (a - np.array([0, 0, -1.]))
        elif mode == 'tilt30': o = np.atleast_1d(0.05 * max(0, a[2] + 0.5))        # >=30 deg below horizontal
        elif mode == 'vert30': o = np.atleast_1d(0.05 * max(0, a[2] + np.cos(np.radians(30))))  # within 30 deg of vertical
        return np.r_[tcp - p, o]
    def ok(a):
        return {'down': a[2] < -np.cos(np.radians(2)), 'tilt30': a[2] <= -0.49, 'vert30': a[2] <= -np.cos(np.radians(31))}[mode]
    kin = False; seeds = [np.array([np.arctan2(p[1], p[0]), -2.0, -1.5, 0.0, 0.0])] + [rng.uniform(lo, hi) for _ in range(n)]
    last = None
    for q0 in seeds:
        r = least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi), max_nfev=300)
        tcp, a, _, _ = fk(r.x)
        if np.linalg.norm(tcp - p) < 0.002 and ok(a):
            kin = True
            h, _ = coll(r.x, B)
            if h is None: return 'ok', r.x
            last = h
    return ('kin:' + str(last)) if kin else 'unreach', None

def jr(q): return None if q is None else np.degrees(q).round(0).tolist()
out = {'s': S_, 'xa': xa, 'task': TASK, 'res': []}
if TASK == 'floor':
    for H in (0.20, 0.25):
        B = to_arm(world_boxes(), H)
        for y in (0.0, 0.10, -0.10, 0.20, -0.20):
            for x in np.round(np.arange(0.05, 0.401, 0.025), 3):
                st, q = ik(np.array([x - xa, y, 0.03 - H]), 'down', B)
                out['res'].append(dict(H=H, x=float(x), y=y, st=st, ext=(ext(q) if q is not None else None)))
            print(H, y, [r['x'] for r in out['res'] if r['H'] == H and r['y'] == y and r['st'] == 'ok'], flush=True)
elif TASK == 'counter':
    for place in ('flush', 'standoff'):
        c = 0.0 if place == 'flush' else xa + 0.07 + 0.02
        for H in (0.85, 0.90, 0.92):
            B = to_arm(world_boxes(counter_edge=c), H)
            sh = static_hits(B)
            for y in (0.0, 0.20, -0.20):
                for dpt in np.round(np.arange(0.05, 0.351, 0.025), 3):
                    if sh: st, q = 'mount-hits-' + '+'.join(sorted(set(sh))), None
                    else: st, q = ik(np.array([c + dpt - xa, y, 0.95 - H]), 'down', B)
                    out['res'].append(dict(place=place, c=c, H=H, x=float(dpt), y=y, st=st, ext=(ext(q) if q is not None else None)))
                print(place, H, y, sh, [r['x'] for r in out['res'] if r.get('place') == place and r['H'] == H and r['y'] == y and r['st'] == 'ok'], flush=True)
elif TASK == 'bins':
    x0, x1 = BIN_X
    for Hb in (0.25, 0.30):
        for H in np.round(np.arange(0.20, 0.901, 0.05), 2):
            B = to_arm(world_boxes(Hb=Hb), H)
            for k, (y0, y1) in BIN_Y.items():
                yc = (y0 + y1) / 2
                for job, mode, dz in (('drop', 'tilt30', WALL + 0.05), ('pick', 'vert30', 0.03), ('pick_down', 'down', 0.03)):
                    sts = []
                    for x in (x1 - 0.05, (x0 + x1) / 2, x0 + 0.05):
                        for y in (yc - 0.04, yc, yc + 0.04):
                            st, q = ik(np.array([x - xa, y, Hb + dz - H]), mode, B, n=40)
                            sts.append(st)
                    out['res'].append(dict(Hb=Hb, H=float(H), bin=k, job=job, n_ok=sts.count('ok'), sts=sts))
                    print(Hb, H, k, job, sts.count('ok'), '/9', flush=True)
elif TASK == 'stow':
    import struct
    def load(pth):
        b = open(pth, 'rb').read()
        if b[:5].lower() == b'solid' and b'facet' in b[:300]:
            return np.array([list(map(float, l.split()[1:4])) for l in b.decode(errors='ignore').splitlines() if l.strip().startswith('vertex')])
        n = struct.unpack('<I', b[80:84])[0]
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
        return a['v'].reshape(-1, 3).astype(float)
    V = {}
    for i, g in enumerate(gm.geometryObjects):
        if g.parentJoint < 1: continue
        v = np.unique(load(g.meshPath).round(4), axis=0); idx = np.linspace(0, len(v) - 1, min(400, len(v))).astype(int)
        V[i] = v[idx] * np.asarray(g.meshScale)
    def extent(q6):
        q = np.zeros(m.nq); q[:6] = q6; q[6] = 0.0; q[7] = 0.0
        pin.updateGeometryPlacements(m, d, gm, gd, q)
        P = np.vstack([gd.oMg[i].act(v.T).T if False else (gd.oMg[i].rotation @ v.T).T + gd.oMg[i].translation for i, v in V.items()])
        return P[:, 0].max() + xa, np.abs(P[:, 1]).max(), P[:, 2].min(), P[:, 2].max()
    lo6 = np.r_[lo, -np.pi]; hi6 = np.r_[hi, np.pi]
    for H in (0.20, 0.30):
        B = to_arm(world_boxes(Hb=0.30), H)
        best = []
        for t in range(6000):
            q6 = rng.uniform(lo6, hi6)
            xm, ym, zm, zM = extent(q6)
            if ym > 0.225 or zm + H < 0.01: continue
            best.append((xm, q6, zM))
        best.sort(key=lambda t: t[0] + 0.3 * t[2])  # prefer low forward extent, then low height
        found = []
        for xm, q6, zM in best[:400]:
            h, _ = coll(q6[:5], B, psis=[q6[5]])
            if h is None:
                found.append((xm, q6, zM));
                if len(found) >= 3: break
        for xm, q6, zM in found:
            print(f'H={H} stow: arm max x = {xm*100:.1f} cm (base front edge = 0; base_link front = {(xa+0.07)*100:.1f}) top z above mount = {zM*100:.0f} cm, q={np.degrees(q6).round(0)}', flush=True)
            out['res'].append(dict(H=H, xmax=float(xm), q=np.degrees(q6).round(0).tolist()))
json.dump(out, open(f'out_chosen/{TASK}_s{S_}.json', 'w'))
