"""Reachability sweep for reBot B601-DM on a mobile base. Usage: python -I sweep.py URDF out.json [margin_deg] [tcp_back]"""
import sys, json, numpy as np, pinocchio as pin
from scipy.optimize import least_squares
URDF, OUT = sys.argv[1], sys.argv[2]
MARGIN = np.deg2rad(float(sys.argv[3]) if len(sys.argv) > 3 else 5.0)
TCP_BACK = float(sys.argv[4]) if len(sys.argv) > 4 else 0.02   # TCP this far behind fingertips (end_link origin)
m = pin.buildModelFromUrdf(URDF); d = m.createData()
FID = m.getFrameId('end_link')
lo = m.lowerPositionLimit[:5] + MARGIN; hi = m.upperPositionLimit[:5] - MARGIN   # q6 = pure roll about approach axis -> fixed 0
rng = np.random.default_rng(0)

def fk(q5):
    q = np.zeros(m.nq); q[:5] = q5
    pin.framesForwardKinematics(m, d, q)
    T = d.oMf[FID]; R = T.rotation; tip = T.translation
    tcp = tip - TCP_BACK * R[:, 0]
    return tcp, R[:, 0], tip, T

# max horizontal reach (fingertip) in URDF, straight-out arm
_, _, tipmax, _ = fk(np.array([0, -np.pi, -np.pi, 0, 0])); RMAX_TIP = float(np.hypot(*tipmax[:2]))
_, _, _, _ = fk(np.zeros(5))
FLANGE_MAX = float(np.hypot(*d.oMi[6].translation[:2]))

def chain_points(q5, psi):
    """Sphere centres (arm frame): links from J2 outward (r=3.5cm), gripper housing box
    (end_link x in [-0.107,-0.073], y +-0.092, z +-0.034, rolled by psi = joint6) and fingers (r=1.5cm)."""
    tcp, a, tip, T = fk(q5)
    J = [d.oMi[i].translation for i in range(2, 7)]  # j2..j6 origins
    Rr = pin.utils.rotate('x', psi)
    def E(v): return T.act(Rr @ np.asarray(v, float))
    pts = []
    seq = J + [E([-0.107, 0, 0])]
    for p0, p1 in zip(seq[:-1], seq[1:]):
        for t in np.linspace(0, 1, 6): pts.append((p0 + t * (p1 - p0), 0.035, 'arm'))
    for x in (-0.107, -0.09, -0.073):
        for y in np.linspace(-0.092, 0.092, 7):
            for z in (-0.034, 0.0, 0.034): pts.append((E([x, y, z]), 0.005, 'grip'))
    for x in np.linspace(-0.073, 0.0, 5):
        for y in (-0.03, 0.03): pts.append((E([x, y, 0]), 0.015, 'finger'))
    return pts

def collides_psi(q5, psi, H, s, task, c):
    """World: floor z=0, robot front edge x=0, arm base at (-s,0,H). Body = box x<=0, z<=H (top at mount plane)."""
    for p, r, tag in chain_points(q5, psi):
        x, y, z = p[0] - s, p[1], p[2] + H
        if tag != 'finger' and z - r < 0.0: return 'floor'
        if x < r and z < H + r: return 'body'
        if task == 'counter':
            if x + r > c and 0.86 - r < z < 0.90 + r: return 'counter'
            if x + r > c + 0.03 and z < 0.86: return 'cabinet'
    return None

MODE = sys.argv[6] if len(sys.argv) > 6 else 'sphere'
CLEAR = 0.01  # required clearance for mesh mode
if MODE == 'mesh':
    import coal
    gm = pin.buildGeomFromUrdf(m, URDF, pin.GeometryType.COLLISION, package_dirs=[])
    gd = gm.createData()
    _req = coal.CollisionRequest(); _req.security_margin = CLEAR
def _boxes(H, s, c, task):
    B = []
    def add(name, xmin, xmax, zmin, zmax):
        ctr = np.array([(xmin + xmax) / 2 + s, 0, (zmin + zmax) / 2 - H])
        B.append((name, coal.Box(xmax - xmin, 4.0, zmax - zmin), coal.Transform3s(np.eye(3), ctr)))
    add('floor', -2, 2, -0.2, 0.0)
    add('body', -1.0, 0.0, -0.2, H - 0.001)   # body box, top at mount plane, front face at x=0, infinitely wide
    if task == 'counter':
        add('counter', c, c + 0.6, 0.86, 0.90); add('cabinet', c + 0.03, c + 0.6, -0.2, 0.86)
    return B
def collides_mesh(q5, H, s, task, c):
    B = _boxes(H, s, c, task)
    for psi in np.linspace(-np.pi, np.pi, 9)[:-1]:
        q = np.zeros(m.nq); q[:5] = q5; q[5] = psi; q[6] = 0.02; q[7] = -0.02
        pin.updateGeometryPlacements(m, d, gm, gd, q)
        hit = None
        for i, g in enumerate(gm.geometryObjects):
            if g.parentJoint < 2: continue
            T = gd.oMg[i]; tf = coal.Transform3s(T.rotation, T.translation)
            for name, shp, bt in B:
                if name == 'floor' and 'finger' in g.name: continue
                res = coal.CollisionResult()
                if coal.collide(g.geometry, tf, shp, bt, _req, res):
                    hit = name; break
            if hit: break
        if hit is None: return None
    return 'all_rolls'
def collides(q5, H, s, task, c):
    if MODE == 'mesh': return collides_mesh(q5, H, s, task, c)
    for psi in (0, np.pi / 2, np.pi / 4, -np.pi / 4):
        if collides_psi(q5, psi, H, s, task, c) is None: return None
    return 'all_rolls'

def solve(p_des, a_des, H, s, task, c, nseeds=40):
    def res(q):
        tcp, a, _, _ = fk(q)
        return np.r_[tcp - p_des, 0.1 * (a - a_des)]
    kin = None
    seeds = [np.clip(np.array(x), lo, hi) for x in ([np.arctan2(p_des[1], p_des[0]), -2.0, -1.5, 0.0, 0.0],
                                                     [np.arctan2(p_des[1], p_des[0]), -2.6, -2.2, -0.5, 0.0],
                                                     [np.arctan2(p_des[1], p_des[0]), -1.2, -1.0, 0.5, 0.0])]
    seeds += [rng.uniform(lo, hi) for _ in range(nseeds)]
    for q0 in seeds:
        r = least_squares(res, q0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=300)
        tcp, a, _, _ = fk(r.x)
        if np.linalg.norm(tcp - p_des) < 0.002 and np.degrees(np.arccos(np.clip(a @ a_des, -1, 1))) < 2.0:
            if kin is None: kin = r.x
            if collides(r.x, H, s, task, c) is None:
                return kin, r.x
    return kin, None

def ext(q):
    tcp = fk(q)[0]; return float(np.linalg.norm(tcp - d.oMi[2].translation))  # 3D J2(shoulder)->TCP

XS = np.round(np.arange(0.05, 0.401, 0.025), 3)
HS = np.round(np.arange(0.15, 0.901, 0.05), 2)
SS = [float(sys.argv[5])]
results = {'RMAX_TIP': RMAX_TIP, 'margin_deg': float(np.degrees(MARGIN)), 'tcp_back': TCP_BACK, 'rows': []}
DOWN = np.array([0, 0, -1.0]); FWD = np.array([1.0, 0, 0])
for s in SS:
    for H in HS:
        c = 0.0 if H < 0.85 else 0.02    # counter edge x (relative to base front edge)
        row = {'s': s, 'H': float(H), 'c': c}
        tasks = {'floor': [(x, y, 0.03, DOWN, 'floor', 0.0) for x in XS for y in (0.0, 0.2, -0.2)],
                 'ctop': [(c + x, y, 0.95, DOWN, 'counter', c) for x in XS for y in (0.0, 0.2, -0.2)],
                 'cside': [(c + x, y, 0.95, FWD, 'counter', c) for x in XS for y in (0.0,)]}
        for name, tl in tasks.items():
            out = []
            for (xw, y, zw, a, task, cc) in tl:
                p = np.array([xw + s, y, zw - H])  # arm frame
                kin, cf = solve(p, a, H, s, task, cc)
                out.append({'x': round(xw - (cc if task == 'counter' else 0), 3), 'y': y, 'kin': kin is not None, 'cf': cf is not None,
                            'ext': (ext(cf) if cf is not None else None), 'ext_kin': (ext(kin) if kin is not None else None), 'q': (np.degrees(cf).round(1).tolist() if cf is not None else None)})
            row[name] = out
        results['rows'].append(row)
        print(s, H, {k: sum(o['cf'] for o in row[k]) for k in tasks}, flush=True)
json.dump(results, open(OUT, 'w'))
